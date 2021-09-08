#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

int lsm6dso_acc_init(void *para1, void *para2);
int lsm6dso_gyro_init(void *para1, void *para2);
int lsm6dso_init_complete(void *para);
int lsm6dso_acc_activate(bool activate);
int lsm6dso_gyro_activate(bool activate);
int lsm6dso_publish_sensor_data(void *para);
int lsm6dso_publish_fake_sensor_data(void *para);
int lsm6dso_acc_config(uint8_t config_type, void *para);
int lsm6dso_gyro_config(uint8_t config_type, void *para);
int lsm6dso_publish_config_resp(void *para);

int ak09918_mag_init(void *para1, void *para2);
int ak09918_mag_init_complete(void *para);
int ak09918_mag_activate(bool activate);
int ak09918_mag_publish_sensor_data(void *para);
int ak09918_mag_config(uint8_t config_type, void *para);
int ak09918_mag_publish_config_resp(void *para);

int bu27030_light_init(void *para, void *para2);
int bu27030_init_complete(void *para);
int bu27030_light_activate(bool activate);
int bu27030_publish_sensor_data(void *para);
int bu27030_config(uint8_t config_type, void *para);
int bu27030_publish_config_resp(void *para);

int bf2092_optical_init(void *para, void *para2);
int bf2092_init_complete(void *para);
int bf2092_optical_activate(bool activate);
int bf2092_publish_sensor_data(void *para);
int bf2092_config(uint8_t config_type, void *para);
int bf2092_publish_config_resp(void *para);

int led_init(void *para, void *para2);
int led_init_complete(void *para);
int led_activate(bool activate);
//int led_publish_sensor_data(void *para);
int led_config(uint8_t config_type, void *para);
int led_publish_config_resp(void *para);

int headled_init(void *para, void *para2);
int headled_init_complete(void *para);
int headled_activate(bool activate);
//int led_publish_sensor_data(void *para);
int headled_config(uint8_t config_type, void *para);
int headled_publish_config_resp(void *para);

int prox_init(void *para1, void *para2);
int prox_init_complete(void *para);
int prox_activate(bool activate);
int prox_publish_sensor_data(void *para);
int prox_config(uint8_t config_type, void *para);
int prox_publish_config_resp(void *para);

int tof_init(void *para1, void *para2);
int tof_init_complete(void *para);
int tof_activate(bool activate);
int tof_publish_sensor_data(void *para);
int tof_config(uint8_t config_type, void *para);
int tof_publish_config_resp(void *para);

int rotv_init(void *para1, void *para2);
int rotv_init_complete(void *para);
int rotv_activate(bool activate);
int rotv_publish_sensor_data(void *para);
int rotv_config(uint8_t config_type, void *para);
int rotv_publish_config_resp(void *para);

int speedv_init(void *para1, void *para2);
int speedv_init_complete(void *para);
int speedv_activate(bool activate);
int speedv_publish_sensor_data(void *para);
int speedv_config(uint8_t config_type, void *para);
int speedv_publish_config_resp(void *para);

#endif
