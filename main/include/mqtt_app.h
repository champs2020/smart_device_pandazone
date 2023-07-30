#ifndef MQTT_H
#define MQTT_H

// Definig MACROS



// interaction with public mqtt broker
int mqtt_app_start(void);
void mqtt_app_publish(char* topic, char *publish_string);
int mqtt_app_subscribe(char* topic, int qos);
#endif