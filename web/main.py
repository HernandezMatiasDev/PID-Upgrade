import ctypes
import math
import platform
import threading
from pathlib import Path
from flask import Flask, render_template, request, jsonify

app = Flask(__name__)

BASE_DIR = Path(__file__).resolve().parent


# 1. Mapear el struct de C (mismo orden y tipos que en pid.h)
class PIDController(ctypes.Structure):
    _fields_ = [
        # Configuración
        ("kp", ctypes.c_float),
        ("ki", ctypes.c_float),
        ("kd", ctypes.c_float),
        ("Tt", ctypes.c_float),
        ("kb", ctypes.c_float),
        ("k_deriv_filter", ctypes.c_float),
        ("out_max", ctypes.c_float),
        ("out_min", ctypes.c_float),
        ("deadband", ctypes.c_float),
        ("rate", ctypes.c_float),
        ("tf_filter", ctypes.c_float),
        # Estado
        ("target", ctypes.c_int32),
        ("ramp_offset", ctypes.c_float),
        ("filt_offset", ctypes.c_float),
        ("integral", ctypes.c_float),
        ("previous_position", ctypes.c_int32),
        ("initialized", ctypes.c_ubyte),
        ("derivative", ctypes.c_float),
    ]


def _find_library() -> Path:
    """Busca la librería compilada al lado de este script, sea cual sea el SO."""
    system = platform.system()
    candidates = {
        "Windows": ["pid.dll"],
        "Darwin": ["pid.dylib", "pid.so"],
    }.get(system, ["pid.so"])

    for name in candidates:
        path = BASE_DIR / name
        if path.exists():
            return path

    raise FileNotFoundError(
        "No se encontro la libreria del PID (pid.dll / pid.so / pid.dylib) "
        f"junto a main.py. Compila pid.c primero (ver build.sh / build.bat). "
        f"Se busco en: {BASE_DIR}"
    )


# 2. Cargar la librería compartida (ruta absoluta, misma carpeta que main.py)
pid_lib = ctypes.CDLL(str(_find_library()))

# 3. Configurar firmas de las funciones
P = ctypes.POINTER(PIDController)
F = ctypes.c_float

pid_lib.PID_Init.argtypes = [P, F, F, F, F, F, F, F, F, F, F]
#                              kp ki kd Tt rate tf kdf max min deadband
pid_lib.PID_Init.restype = None

pid_lib.PID_Compute.argtypes = [P, ctypes.c_int32, F, F]  # pid, posicion, dt, feedforward
pid_lib.PID_Compute.restype = F

pid_lib.PID_SetTarget.argtypes = [P, ctypes.c_int32]
pid_lib.PID_SetGains.argtypes = [P, F, F, F, F, F, F, F]  # kp ki kd Tt rate tf kdf
pid_lib.PID_SetOutputLimits.argtypes = [P, F, F]  # out_max, out_min
pid_lib.PID_SetDeadband.argtypes = [P, F]
pid_lib.PID_SetIntegral.argtypes = [P, F]
pid_lib.PID_Reset.argtypes = [P]

# Instancia global del PID para el simulador.
# Valores por defecto: TODOS los filtros desactivados (0 = desactivado en pid.c):
#   - Tt = 0             -> anti-windup (back-calculation) desactivado
#   - rate = 0           -> sin rampa de setpoint
#   - tf_filter = 0      -> sin filtro pasa-bajos de setpoint
#   - k_deriv_filter = 0 -> sin filtro de la derivada
# (el frontend manda los valores reales apenas carga la pagina)
pid = PIDController()
pid_lock = threading.Lock()  # Flask corre con threaded=True y el struct es compartido
pid_lib.PID_Init(
    ctypes.byref(pid),
    0.01, 0.0, 0.0,      # kp, ki, kd
    0.0,                 # Tt
    0.0, 0.0, 0.0,       # rate, tf_filter, k_deriv_filter
    300.0, -300.0,       # out_max, out_min
    0.0,                 # deadband
)
pid_lib.PID_SetTarget(ctypes.byref(pid), 0)  # 0 = centro del riel (arriba positivo, abajo negativo)


def _safe(v):
    """JSON no admite NaN/Infinity: si algo se disparó, lo convertimos a un
    número finito en vez de mandar un JSON inválido que rompería el
    fetch().json() del frontend y congelaría la animación."""
    if isinstance(v, float) and not math.isfinite(v):
        return 0.0
    return v


def _current_error():
    """pid.c no guarda el error, asi que lo reconstruimos igual que en
    PID_Compute (con la posicion del ultimo Compute y el offset ya filtrado)."""
    error = float(pid.target - pid.previous_position) + pid.filt_offset
    if error > pid.deadband:
        error -= pid.deadband
    elif error < -pid.deadband:
        error += pid.deadband
    else:
        error = 0.0
    return error


def _state_is_finite():
    return all(
        math.isfinite(v)
        for v in (pid.integral, pid.derivative, pid.ramp_offset, pid.filt_offset)
    )


def snapshot(output=None, dt=None):
    """Devuelve todas las variables del struct + telemetria extra."""
    data = {
        # Configuración
        "kp": _safe(pid.kp),
        "ki": _safe(pid.ki),
        "kd": _safe(pid.kd),
        "Tt": _safe(pid.Tt),
        "kb": _safe(pid.kb),
        "k_deriv_filter": _safe(pid.k_deriv_filter),
        "out_max": _safe(pid.out_max),
        "out_min": _safe(pid.out_min),
        "deadband": _safe(pid.deadband),
        "rate": _safe(pid.rate),
        "tf_filter": _safe(pid.tf_filter),
        # Estado
        "target": pid.target,
        "ramp_offset": _safe(pid.ramp_offset),
        "filt_offset": _safe(pid.filt_offset),
        "integral": _safe(pid.integral),
        "previous_position": pid.previous_position,
        "derivative": _safe(pid.derivative),
        # Telemetria derivada
        "error": _safe(_current_error()),
    }
    if output is not None:
        data["output"] = _safe(output)
    if dt is not None:
        data["dt"] = dt
    return data


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/compute', methods=['POST'])
def compute():
    try:
        data = request.json
        dt = float(data['dt'])
        # Clampeamos dt para evitar picos (ej: pestaña en segundo plano) que
        # dispararian la derivada a valores absurdos.
        dt = max(0.0, min(dt, 0.1))
        feedforward = float(data.get('feedforward', 0.0))
        with pid_lock:
            output = pid_lib.PID_Compute(
                ctypes.byref(pid), int(data['position']), dt, feedforward
            )
            if not math.isfinite(output) or not _state_is_finite():
                # Ante un valor invalido, no dejamos que el estado del PID quede
                # corrompido: lo reseteamos para que el sistema se recupere solo
                # en vez de quedar tildado mandando NaN para siempre.
                pid_lib.PID_Reset(ctypes.byref(pid))
                pid.ramp_offset = 0.0
                pid.filt_offset = 0.0
                output = 0.0
            return jsonify(snapshot(output=output, dt=dt))
    except Exception as exc:
        return jsonify({"error": str(exc)}), 400


@app.route('/update', methods=['POST'])
def update_params():
    try:
        data = request.json

        with pid_lock:
            pid_lib.PID_SetTarget(ctypes.byref(pid), int(data['target']))
            pid_lib.PID_SetGains(
                ctypes.byref(pid),
                float(data['kp']),
                float(data['ki']),
                float(data['kd']),
                float(data.get('Tt', pid.Tt)),
                float(data.get('rate', pid.rate)),
                float(data.get('tf_filter', pid.tf_filter)),
                float(data.get('k_deriv_filter', pid.k_deriv_filter)),
            )
            pid_lib.PID_SetOutputLimits(
                ctypes.byref(pid),
                float(data.get('out_max', pid.out_max)),
                float(data.get('out_min', pid.out_min)),
            )
            pid_lib.PID_SetDeadband(
                ctypes.byref(pid),
                float(data.get('deadband', pid.deadband)),
            )

            if data.get('reset', False):
                pid_lib.PID_Reset(ctypes.byref(pid))

            return jsonify(snapshot())
    except Exception as exc:
        return jsonify({"error": str(exc)}), 400


if __name__ == '__main__':
    # threaded=True evita que un pedido lento (p.ej. mientras se arrastra un
    # slider) bloquee el polling continuo de /compute con el servidor de
    # desarrollo de Flask, que por defecto es de un solo hilo.
    app.run(debug=True, threaded=True)
