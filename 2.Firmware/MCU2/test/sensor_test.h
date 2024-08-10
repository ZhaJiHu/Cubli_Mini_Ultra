// void setup()
// {
//     Serial.begin(115200);
//     sensor->init(&spi_);
//     sensor2.init(&spi_, true);
//     sensor3.init(&spi_, true);

//     current_sense->init();
//     current_sense->gain_a *= -1;

//     current_sense2.init();
//     current_sense2.gain_a *= -1;

//     current_sense3.init();
//     current_sense3.gain_a *= -1;
// }

// void loop()
// {
//     // sensor->update();
//     // sensor2.update();
//     // sensor3.update();
//     // printf("motor1 angle: %0.3f vel: %0.3f motor2 angle: %0.3f vel: %0.3f motor3 angle: %0.3f
//     vel: %0.3f\r\n",
//     //     sensor->getAngle(),
//     //     sensor->getVelocity(),
//     //     sensor2.getAngle(),
//     //     sensor2.getVelocity(),
//     //     sensor3.getAngle(),
//     //     sensor3.getVelocity());

//     PhaseCurrent_s currents1 = current_sense->getPhaseCurrents();
//     float current_magnitude1 = current_sense->getDCCurrent();
//     PhaseCurrent_s currents2 = current_sense2.getPhaseCurrents();
//     float current_magnitude2 = current_sense2.getDCCurrent();
//     PhaseCurrent_s currents3 = current_sense3.getPhaseCurrents();
//     float current_magnitude3 = current_sense3.getDCCurrent();

//     printf("motor1 a[%0.3f] mA b[%0.3f] mA c[%0.3f] mA cur[%0.3f] mA\n",
//         (currents1.a*-1000),
//         (currents1.b*1000),
//         (currents1.c*1000),
//         (current_magnitude1*1000));

//      printf("motor2 a[%0.3f] mA b[%0.3f] mA c[%0.3f] mA cur[%0.3f] mA\n",
//         (currents2.a*-1000),
//         (currents2.b*1000),
//         (currents2.c*1000),
//         (current_magnitude2*1000));

//      printf("motor3 a[%0.3f] mA b[%0.3f] mA c[%0.3f] mA cur[%0.3f] mA\n",
//         (currents3.a*-1000),
//         (currents3.b*1000),
//         (currents3.c*1000),
//         (current_magnitude3*1000));
//     delay(10);
// }