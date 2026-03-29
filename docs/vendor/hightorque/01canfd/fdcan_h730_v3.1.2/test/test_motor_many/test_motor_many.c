#include "test_motor_many.h"


void test_time_out(int16_t t_ms)
{
    for (uint8_t id = 1; id <= MANY_MOTOR_SIZE; id++)
    {
        motor_many_time_out(PORT1, id, t_ms);
    }

    motor_many_send(PORT1, MANY_GET_MODE_FLAUT_POS_VEL_TQE);
    motor_many_send(PORT1, MANY_GET_MODE_FLAUT_POS_VEL_TQE);
    motor_many_send(PORT1, MANY_GET_MODE_FLAUT_POS_VEL_TQE);
}


void test_motor_many()
{
    const uint8_t mode = 3;

    for (uint8_t id = 1; id <= MANY_MOTOR_SIZE; id++)
    {
        switch (mode)
        {
        case 0:
            motor_many_dq_volt(PORT1, id, 1);
            break;
        case 1:
            motor_many_dq_current(PORT1, id, 1);
            break;
        case 2:
            motor_many_pos(PORT1, id, 1);
            break;
        case 3:
            motor_many_vel(PORT1, id, 0.1);
            break;
        case 4:
            motor_many_tqe(PORT1, id, 1.0f);
            break;
        case 5:
            motor_many_pos_vel(PORT1, id, 1, 0.1);
            break;
        case 6:
            motor_many_pos_vel_MAXtqe(PORT1, id, 0, 0.1, NAN_FLOAT);
            break;
        case 7:
            motor_many_pos_vel_acc(PORT1, id, 1, 1, 0.2);
            break;
        case 8:
            motor_many_pos_vel_tqe_kp_kd(PORT1, id, 1, 0.1, 0, 1, 1);
            break;
        case 9:
            motor_many_pos_vel_tqe_kp_kd_2(PORT1, id, 1, 0.1, 0, 1, 1);
            break;
        default:
            break;
        }
    }

    motor_many_send(PORT1, MANY_GET_TEMP_FLAUT_POS_VEL_TQE);
}
