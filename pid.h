#ifndef PID_H
#define PID_H
#include <stdint.h>
typedef struct {
    // Variables de configuración
    float kp; // Ganancia proporcional, reacciona al error actual
    float ki; // Ganancia integral (0 = sin integral)
    float kd; // Ganancia derivativa: frena según la velocidad (0 = sin derivada)
    float Tt; // Constante de tiempo del anti-windup (s): qué tan rápido se descarga la integral cuando la salida está saturada (0 = sin anti-windup)
    float kb;  // 1/Tt, calculada sola. es para evitar division por cero en el cálculo de la integral y que el 0 sea sin anti-windup
    float k_deriv_filter; // Constante de tiempo (s) del filtro de la derivada, suaviza el ruido (0 = sin filtro)
    float out_max; // Valor máximo de salida
    float out_min; // Valor mínimo de salida
    float deadband; // Rango de error en el que no se aplica correccion para evitar oscilaciones
    float rate;   // Rampa: velocidad máxima a la que se mueve el objetivo, en unidades/s (0 = sin rampa) (termino proporcional)
    float tf_filter; // Constante de tiempo (s) del filtro que redondea el objetivo (0 = sin filtro) (termino proporcional)

    // Variables de estado
    int32_t target; // Objetivo real
    float ramp_offset; // Distancia entre la salida de la rampa y el target (termino proporcional)(rampa)
    float filt_offset; // Distancia entre la salida del filtro y el target; es lo que se suma al error (termino proporcional)(filtro)(viene despues de la rampa)
    float integral;  // Término integral ya multiplicado por Ki (unidades de salida)
    int32_t previous_position;  // Posición del ciclo anterior, para calcular la velocidad
    unsigned char initialized;  // 1 = hay que inicializar (primer ciclo o en un Reset)
    float derivative; // Término derivativo, ya filtrado (sin multiplicar por Kd))
} PIDController;

// Configura todo y deja el estado en cero
void PID_Init(PIDController *pid, float kp, float ki, float kd, float Tt, float rate, float tf_filter, float k_deriv_filter, float out_max, float out_min, float deadband);

// Calcula la salida del PID. Se llama en cada ciclo de control
float PID_Compute(PIDController *pid, int32_t current_position, float delta_time, float Feedforward);

// Cambia el objetivo; la rampa y el filtro lo siguen sin saltos
void PID_SetTarget(PIDController *pid, int32_t new_target);

// Borra la integral y la derivada; el próximo ciclo reinicia la rampa y el filtro
void PID_Reset(PIDController *pid);

// Fuerza el valor de la integral (en unidades de salida)
void PID_SetIntegral(PIDController *pid, float integral);

// Cambia los límites de salida (si estan invertidos, los intercambia)
void PID_SetOutputLimits(PIDController *pid, float out_max, float out_min);

// Cambia todas las ganancias y tiempos de una vez
void PID_SetGains(PIDController *pid, float kp, float ki, float kd, float Tt, float rate, float tf_filter, float k_deriv_filter);

// Cambia Tt (anti-windup); recalcula kb
void PID_SetTt(PIDController *pid, float Tt);

// Cambia el filtro de la derivada (no acepta negativos)
void PID_SetDerivFilter(PIDController *pid, float k_deriv_filter);

// Cambia la zona muerta (no acepta negativos)
void PID_SetDeadband(PIDController *pid, float deadband);

// Cambia Ki; si es 0 borra la integral y recalcula kb
void PID_SetKi(PIDController *pid, float ki);

// Cambia Kp
void PID_SetKp(PIDController *pid, float kp);

// Cambia Kd
void PID_SetKd(PIDController *pid, float kd);

// Cambia la velocidad de la rampa (no acepta negativos)
void PID_SetRate(PIDController *pid, float rate);

// Cambia el filtro del objetivo (no acepta negativos)
void PID_SetTfFilter(PIDController *pid, float tf_filter);

#endif // PID_H