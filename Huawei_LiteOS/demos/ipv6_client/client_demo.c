
#ifdef WITH_LWIP


#include "lwip/sockets.h"
#include "los_task.h"


void ipv6_tcp_test(void)
{
    struct sockaddr_in6 server_addr;
    char msg[128];    
    int rbytes = -1;
    
    //hieth_hw_init();
    //net_init();
    
    int client_fd = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);
    if (client_fd < 0)
    {
        printf("client_fd is %d\n",client_fd);
        return;
    }

    server_addr.sin6_family = AF_INET6;    
    server_addr.sin6_port = htons(1883);    
    if (inet_pton(AF_INET6, "fe80::70d7:3d63:ca2d:ee52", &server_addr.sin6_addr) <= 0)    
    {        
        printf("inet_pton error!!!\n"); 
        return;
    }
    if (connect(client_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)   
    {        
        printf("socket connect error=%d(%s)!!!\n", errno, strerror(errno)); 

        return;  
    }   
    printf("connect to server ok!\n");

    close(client_fd);
    #if 0
    while ( (rbytes = read(client_fd, msg, sizeof(msg)-1)) > 0)    
    {       
        msg[rbytes] = 0;
        printf("%s\n", msg);
        write(client_fd, "receive ok", strlen("receive ok"));
    }   
    
    if (rbytes < 0)  
    {        
        printf("read error=%d(%s)!!!\n", errno, strerror(errno));    
    }
    #endif

}

void ipv6_udp_test(void)
{

    struct sockaddr_in6 server_addr;
    char msg[128];    
    int rbytes = -1; 
    
    //hieth_hw_init();
    //net_init();
    
    int client_fd = socket(AF_INET6, SOCK_DGRAM, 17);
    if (client_fd < 0)
    {
        printf("client_fd is %d\n",client_fd);
        return;
    }

    server_addr.sin6_family = AF_INET6;    
    server_addr.sin6_port = htons(3500);    
    if (inet_pton(AF_INET6, "fe80::70d7:3d63:ca2d:ee52", &server_addr.sin6_addr) <= 0)    
    {        
        printf("inet_pton error!!!\n"); 
        return;
    }
   
    if (connect(client_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)   
    {        
        printf("socket connect error=%d(%s)!!!\n", errno, strerror(errno)); 

        return;  
    }   
    printf("connect to server ok!\n");


    int aa  = write(client_fd, "receive ok", strlen("receive ok"));
    //int aa = sendto(client_fd, "receive ok", strlen("receive ok"), 0, (struct sockaddr *)&server_addr, sizeof(server_addr));

    printf("aa is %d\n",aa);
    close(client_fd);
    
    #if 0
    while ( (rbytes = read(client_fd, msg, sizeof(msg)-1)) > 0)    
    {       
        msg[rbytes] = 0;
        printf("%s\n", msg);
        write(client_fd, "receive ok", strlen("receive ok"));
    }   
    
    if (rbytes < 0)  
    {        
        printf("read error=%d(%s)!!!\n", errno, strerror(errno));    
    }
    #endif
}


#endif
