#include "transmission.h"


void tcp_client_task(void *pvParameters)
{
//    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    int sock, err, ackConnect = 0;
    #ifdef CONFIG_EXAMPLE_IPV4
            struct sockaddr_in destAddr;
            destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
            destAddr.sin_family = AF_INET;
            destAddr.sin_port = htons(PORT);
            addr_family = AF_INET;
            ip_protocol = IPPROTO_IP;
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
					break;
				}
			}
    	}

//        char msj[] = "Hola viejo\r\n";
//        sprintf(payload2,"%s",msj);
//    	char msj[100];
    	sprintf(payload2,"%s,%s,%s,%s,%s,%s\r\n",temp_string_dht22,temp_string_aht10,rh_string_dht22,rh_string_aht10,temp_string_bmp280,pressure_string_bmp280);
        err = send(sock, payload2, strlen(payload2), 0);
		if (err < 0) {
			ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
			break;
		}
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
//    vTaskDelete(NULL);
}
