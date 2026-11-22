#include "pid.h"

// Recalcula kb = 1/Tt
// Queda en 0 (anti-windup apagado) si ki = 0 o Tt <= 0
static void pid_update_kb(PIDController *pid)
{
    pid->kb = (pid->ki != 0.0f && pid->Tt > 0.0f) ? 1.0f / pid->Tt : 0.0f;
}

// Setea Kp (ganancia proporcional)
void PID_SetKp(PIDController *pid, float kp)
{
    pid->kp = kp;
}

// Setea Ki. Con Ki = 0 borra la integral y recalcula kb
void PID_SetKi(PIDController *pid, float ki)
{
    pid->ki = ki;
    if (ki == 0.0f) pid->integral = 0.0f;  
    pid_update_kb(pid);
}

// Setea Kd (ganancia derivativa)
void PID_SetKd(PIDController *pid, float kd)
{
    pid->kd = kd;
}

// Setea Tt (anti-windup) y recalcula kb
// Tt <= 0 => anti-windup desactivado
void PID_SetTt(PIDController *pid, float Tt)
{
    pid->Tt = Tt;
    pid_update_kb(pid);
}

// Setea la constante de tiempo del filtro del objetivo 
// (0 = sin filtro), Los negativos se hacen 0 (termino proporcional)
void PID_SetTfFilter(PIDController *pid, float tf_filter)
{
    pid->tf_filter = (tf_filter > 0.0f) ? tf_filter : 0.0f; 
}

// Setea la velocidad máxima de la rampa (0 = sin rampa) 
// Los negativos se hacen 0 (termino proporcional)
void PID_SetRate(PIDController *pid, float rate)
{
    pid->rate = (rate > 0.0f) ? rate : 0.0f;  
}

// Guarda el filtro de la derivada, Los negativos se hacen 0, (0 = sin filtro)
void PID_SetDerivFilter(PIDController *pid, float k_deriv_filter) // un numero negativo puede generar una division por cero
{
    pid->k_deriv_filter = (k_deriv_filter > 0.0f) ? k_deriv_filter : 0.0f;  
}

// Guarda la zona muerta
// Un negativo no tiene sentido, se hace 0
void PID_SetDeadband(PIDController *pid, float deadband) 
{
    pid->deadband = (deadband > 0.0f) ? deadband : 0.0f;
}

// Valor absoluto entero 
static inline int32_t int_abs(int32_t val) 
{
    return (val < 0) ? -val : val;
}

// Limita 'value' al rango [min, max]
static inline float clamp(float value, float min, float max) 
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

// Configura todas las ganancias y tiempos llamando a cada setter
// los seters se encargan de los casos especiales (como Ki = 0, Tt <= 0, etc)
void PID_SetGains(PIDController *pid, float kp, float ki, float kd, float Tt, float rate, float tf_filter, float k_deriv_filter)
{
    PID_SetKp(pid, kp);
    PID_SetKi(pid, ki);
    PID_SetKd(pid, kd);
    PID_SetTt(pid, Tt);
    PID_SetRate(pid, rate);
    PID_SetTfFilter(pid, tf_filter);
    PID_SetDerivFilter(pid, k_deriv_filter);
}

// Guarda los límites de salida; si vienen invertidos los intercambia
void PID_SetOutputLimits(PIDController *pid, float out_max, float out_min)
{
    if (out_min > out_max)  
    {
        float tmp = out_max;
        out_max = out_min;
        out_min = tmp;
    }
    pid->out_max = out_max;
    pid->out_min = out_min;
}

// Deja el estado en cero y aplica la configuración. 
// La rampa y el filtro se inicializan en el primer Compute
// Es la función que se llama al principio para configurar el PID.
void PID_Init(PIDController *pid, float kp, float ki, float kd, float Tt, float rate, float tf_filter,
              float k_deriv_filter, float out_max, float out_min, float  deadband)
{
    // Estado
    pid->target = 0;
    pid->integral = 0.0f;
    pid->previous_position = 0;
    pid->initialized = 1;
    pid->derivative = 0.0f;
    pid->ramp_offset = 0.0f;
    pid->filt_offset = 0.0f;

    // para no tener error en (pid_update_kb )
    pid->ki = 0.0f;
    pid->Tt = 0.0f;

    // Configuración
    PID_SetGains(pid, kp, ki, kd, Tt, rate, tf_filter, k_deriv_filter);
    PID_SetOutputLimits(pid, out_max, out_min);
    PID_SetDeadband(pid, deadband);
}

// Cambia el objetivo
// Suma el corrimiento a los offsets para que la rampa y el filtro sigan en el mismo punto absoluto.
void PID_SetTarget(PIDController *pid, int32_t new_target)
{
    float shift = (float)(pid->target - new_target);
    pid->ramp_offset += shift;
    pid->filt_offset += shift;
    pid->target = new_target;
}

// Borra integral y derivada
// initialized = 1 hace que el próximo Compute reinicie rampa filtro y posición previa
void PID_Reset(PIDController *pid)
{
    pid->integral = 0.0f;
    pid->initialized = 1;
    pid->derivative = 0.0f;
}

// Fuerza el valor de la integral
void PID_SetIntegral(PIDController *pid, float integral)
{
    pid->integral = integral;
}


// Calcula la salida del PID. Se llama en cada ciclo con la posición actual y el tiempo transcurrido (s)
// Feedforward es una salida extra que se suma al PID
float PID_Compute(PIDController *pid, int32_t current_position, float delta_time, float Feedforward)
{

    // Primer ciclo: la rampa y el filtro empiezan en la posición actual, así no hay salto
    if (pid->initialized)
    {
        pid->previous_position = current_position;
        pid->ramp_offset = (float)(current_position - pid->target);
        pid->filt_offset = pid->ramp_offset;
        pid->initialized = 0;
    }

    // Calculamos p 

    // Rampa: acerca el offset a 0 como máximo 'rate * dt' por ciclo (limita la velocidad)
    if (pid->rate > 0.0f) 
    {
        float step = pid->rate * delta_time;

        if (pid->ramp_offset > step)      
            pid->ramp_offset -= step;
            else if(pid->ramp_offset < -step) 
                pid->ramp_offset += step;
            else                   
                pid->ramp_offset = 0.0f;
    } 
    else 
    {
        pid->ramp_offset = 0.0f;
    }

    // filtro pasa-bajos de primer orden (sobre la salida de la rampa)
    if (pid->tf_filter > 0.0f)
    {
        float a = delta_time / (pid->tf_filter + delta_time);
        pid->filt_offset += a * (pid->ramp_offset - pid->filt_offset);
    }
    else
    {
        pid->filt_offset = pid->ramp_offset;
    }

    // Error (con offset de rampa/filtro): se usan en P, I y la deadband.
    float error = (float)(pid->target - current_position) + pid->filt_offset;


    // Zona muerta: ignora errores chicos
    // resta el ancho de la banda a los grandes
    if (error > pid->deadband)      
    {
        error = error - pid->deadband;
    }
    else if (error < -pid->deadband) 
    {
        error = error + pid->deadband;
    }
    else 
    {
        error = 0;
    }


    // calculamos la derivada

    // calculamos la derivada con filtro de paso bajo
    if (delta_time > 1e-6f)
    {
        float raw_derivative = ( pid->previous_position - current_position) / delta_time;

        float alpha = delta_time / (pid->k_deriv_filter + delta_time);

        pid->derivative = (alpha * raw_derivative) + ((1.0f - alpha) *  pid->derivative);
    }

    pid->previous_position = current_position;



    // calcular salida pid
    float raw_output = (pid->kp * error) + (pid->integral) + (pid->kd * pid->derivative) + Feedforward;
    float output = clamp(raw_output, pid->out_min, pid->out_max);
    

    // Calculamos la integral: acumula Ki*error; el término con kb la descarga cuando la salida satura (anti-windup)
    pid->integral += (pid->ki * error + pid->kb * (output - raw_output)) * delta_time;
    pid->integral = clamp(pid->integral, pid->out_min - Feedforward, pid->out_max - Feedforward);

   
    return output;
}