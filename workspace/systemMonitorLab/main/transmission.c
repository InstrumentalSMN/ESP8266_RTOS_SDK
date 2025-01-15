#include "transmission.h"

void encodeMessage126(uint8_t * buf, uint8_t * message,size_t message_len){
	size_t buf_len = strlen((const char*)buf);
	memset(message, 0, message_len);
	message[0] = 0x81; // Opcode 0x1 y datos enmascarados
	//	uint16_taux = ((uint16_t)strlen(aux)) | 0x8000;
	message[1] = 0xFE;
	uint16_t largo = ((uint16_t)buf_len);
	message[2] = (largo & 0xFF00)>>8;
	message[3] = (largo & 0x00FF)>>0;
	uint32_t mask_key = 0x12345678; // Clave de codificación
	//Copio la clave
	message[4] = (mask_key & 0xFF000000)>>24;	//0x12
	message[5] = (mask_key & 0x00FF0000)>>16;	//0x34
	message[6] = (mask_key & 0x0000FF00)>>8;	//0x56
	message[7] = (mask_key & 0x000000FF)>>0;	//0x78
	//	uint32_t mask_key_aux = (mask_key & 0x00FF0000)>>16;
	memcpy(message + 8, buf, buf_len);
	//	message[6] = message[6]^0x12;
	//	message[7] = message[7]^0x34;
	//	// Copiar los datos sin enmascarar
	int value = 4;
	for (int i = 8; i < buf_len + 8; i++) {
	//		uint8_t a = ((uint8_t*)&mask_key)[i % 4];
	//	    message[i] ^= ((uint8_t*)&mask_key)[i % 4]; // Aplicar XOR con la clave de codificación
		message[i] ^= message[value];
		value++;
		if(value > 7){
			value = 4;
		}
	}




}


void encodeMessage125(uint8_t * buf, uint8_t * message, size_t message_len){
		size_t buf_len = strlen((const char*)buf);
		memset(message, 0, message_len);
		message[0] = 0x81; // Opcode 0x1 y datos enmascarados
		//	uint16_taux = ((uint16_t)strlen(aux)) | 0x8000;
		message[1] = ((uint8_t)buf_len) | 0x80; // Longitud de los datos y seteo el bit de enmascaramiento
		uint32_t mask_key = 0x12345678; // Clave de codificación
		//Copio la clave
		message[2] = (mask_key & 0xFF000000)>>24;	//0x12
		message[3] = (mask_key & 0x00FF0000)>>16;	//0x34
		message[4] = (mask_key & 0x0000FF00)>>8;	//0x56
		message[5] = (mask_key & 0x000000FF)>>0;	//0x78
	//	uint32_t mask_key_aux = (mask_key & 0x00FF0000)>>16;
		memcpy(message + 6, buf, buf_len);
	//	message[6] = message[6]^0x12;
	//	message[7] = message[7]^0x34;
	//	// Copiar los datos sin enmascarar
		int value = 2;
		for (int i = 6; i < buf_len + 6; i++) {
	//		uint8_t a = ((uint8_t*)&mask_key)[i % 4];
	//	    message[i] ^= ((uint8_t*)&mask_key)[i % 4]; // Aplicar XOR con la clave de codificación
			message[i] ^= message[value];
			value++;
			if(value > 5){
				value = 2;
			}
		}


}

void tcp_client_task(void *pvParameters)
{
//    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    int sock, err, ackConnect= 0;
    uint32_t counter = 0;
    #ifdef CONFIG_EXAMPLE_IPV4
            struct sockaddr_in destAddr;
            destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
            destAddr.sin_family = AF_INET;
            destAddr.sin_port = htons(PORT);
            addr_family = AF_INET;
            ip_protocol = IPPROTO_TCP;
            inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
    #else // IPV6
            struct sockaddr_in6 destAddr;
            inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
            destAddr.sin6_family = AF_INET6;
            destAddr.sin6_port = htons(PORT);
            destAddr.sin6_scope_id = tcpip_adapter_get_netif_index(TCPIP_ADAPTER_IF_STA);
            addr_family = AF_INET6;
            ip_protocol = IPPROTO_IPV6;
            inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
    #endif

    while (1) {

    	if(ackConnect != 1){
			while(1){
				sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
				if (sock < 0) {
					ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
		//                break;
				}
				ESP_LOGI(TAG, "Socket created");
				err = connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
				if (err != 0) {
					ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
					ackConnect = 0;
					close(sock);
		//                continue;
				}else{
					ESP_LOGI(TAG, "Successfully connected");
					ackConnect = 1;
			//    	char aux[200] = "{\'message\':\'hola\'}";
					char host[] = "10.10.13.180";
					uint16_t server_port = 8000;
					char path[] = "/ws/environment-monitoring-system-server/";
					char key[] = "x3JJHMbDL1EzLkh9GBhXDw==";
					char header[256];
					sprintf(header,	"GET %s HTTP/1.1\r\n"
									"Host: %s:%d\r\n"
									"Upgrade: websocket\r\n"
									"Connection: Upgrade\r\n"
									"Sec-WebSocket-Key: %s\r\n"
									"Sec-WebSocket-Version: 13\r\n"
									"\r\n", path, host, server_port, key);
//					int32_t a = strlen(header);



			//		int32_t len = send(WEB_SOCK, header, strlen(header));
			//		if (a != len){ //valido que se envie correctamente
			//			printf("Mensaje no enviado\r\n");
			//			gpioWrite( GPIO8, ON );
			//			gpioWrite( GPIO7, OFF );
			//			return ERROR;
			//		}


			//    	sprintf(payloadWebSocket,"i,%d,%s,%s,%s,%s,%s,%s,\r\n",counter,temp_string_dht22,temp_string_aht10,temp_string_bmp280,rh_string_dht22,rh_string_aht10,pressure_string_bmp280);
					err = send(sock, header, strlen(header), 0);
					if (err < 0) {
						ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
						break;
					}else{
						ESP_LOGI(TAG, "Envie correctamente el header de WEB SOCKET\r\n");
					}

					break;
				}
			}
    	}
//    	char aux[] = "{\"message\":\"\"}";
//    	char aux[] = "{\"message\":\"hola\"}";
    	char aux[] = "{\"message\":\"holamundobuenasholamundobuenasholamundobuenasholamundobuenasholamundobuenasholamundobuenasholamundobuenasholamundobuenasholab6\"}";
    	char message[140];

//    	encodeMessage125((uint8_t * )aux,(uint8_t * )message,sizeof(message));
//    	ESP_LOGI(TAG, "MensajeSINCodificar: %s",aux);
//    	ESP_LOGI(TAG, "MensajeCodificado: %s",message);
//
//    	err = send(sock, message, strlen(aux)+6, 0);

    	encodeMessage126((uint8_t * )aux,(uint8_t * )message,sizeof(message));
    	ESP_LOGI(TAG, "MensajeSINCodificar: %s",aux);
    	ESP_LOGI(TAG, "MensajeCodificado: %s",message);

    	err = send(sock, message, strlen(aux)+8, 0);

    	if (err < 0) {
			ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
			break;
		}


//        char msj[] = "Hola viejo\r\n";
//        sprintf(payload2,"%s",msj);
//    	char msj[100];
//    	sprintf(payload2,"i,%d,%s,%s,%s,%s,%s,%s,\r\n",counter,temp_string_dht22,temp_string_aht10,temp_string_bmp280,rh_string_dht22,rh_string_aht10,pressure_string_bmp280);
//        err = send(sock, payload2, strlen(payload2), 0);

//		if (err < 0) {
//			ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
//			break;
//		}
		counter = counter + 1;
        vTaskDelay(TIME_TRANSMISSION*1000 / portTICK_PERIOD_MS);
    }
//    vTaskDelete(NULL);
}
