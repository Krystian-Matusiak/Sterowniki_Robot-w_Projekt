#include <iostream>
#include <iomanip>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <typeinfo>
#include <cstdlib>
#include <string>
#include <errno.h>
#include <pthread.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>


#define PORT_REC          6001
#define PORT_TRAN           6000

#define ROZM_TRAN          30
#define ROZM_REC           10

#define ZN_KROTKI             4
#define ZN_DLUGI              8
#define MARGINES             10
#define ODSTEP_ZNAKU         16
#define PRZESUNIECIE_X        3
#define WSKSEK_D              8

using namespace std;


static const int CHANNEL = 0;

uint8_t TX[11];
pthread_mutex_t locker;


















///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////























/*! \brief Klasa odpowiadająca za nadawanie danych do innego urządzenia. */

class Transmitter{

public:

    /*! Funkcja odpowiadająca za nadawanie danych poprzez socket.
        \param GnOdbioru - deskryptor gniazda.
     */
    int Nadaj(int Sk2Client, char DANA[100]);

    /*! Funkcja aktywująca stan "nasłuchiwania". Gdy dane się pojawią, aktywowana jest funkcja Odbierz, aby przyjąć dane.
     */
    int SocketConfig(int Port , uint8_t * buffer);


    char DANA[ROZM_TRAN];


    // Kolejne dane to PWM1 DIR1 PWM2 DIR2 BATTERY oraz MODE (1-tryb manualny, 0-tryb automatyczny)
    int tab_danych[6];

};




int Transmitter::Nadaj(int Sk2Client, char DANA[100]){  //const  char Napis[8][301])
  ssize_t  IlWyslanych;
  ssize_t IlDoWyslania = ROZM_TRAN*sizeof(char);

    IlWyslanych = write(Sk2Client , DANA, ROZM_TRAN*sizeof(char) );

    if (IlWyslanych < 0){
    cerr<<"***ERROR*****"<<endl;
  }

}


int Transmitter::SocketConfig(int Port , uint8_t * buffer){

    int                 GnNadawania;
    struct sockaddr_in  DaneAdSerw;

    bzero((char *)&DaneAdSerw,sizeof(DaneAdSerw));
    DaneAdSerw.sin_family = AF_INET;
    DaneAdSerw.sin_addr.s_addr = inet_addr("192.168.8.100");
    //DaneAdSerw.sin_addr.s_addr = inet_addr("10.11.60.42");
    DaneAdSerw.sin_port = htons(Port);

    GnNadawania = socket(AF_INET,SOCK_STREAM,0);
    if (GnNadawania < 0) {
       cerr << "*** Blad otwarcia gniazda." << endl;
       return 1;
    }
    if (connect(GnNadawania,(struct sockaddr*)&DaneAdSerw,sizeof(DaneAdSerw)) < 0){
       cerr << "*** Brak mozliwosci polaczenia do portu: " << Port << endl;
       return 1;
     }

    // Dane po kolei: PWM1 DIR1 PWM2 DIR2 BATT
    char DANA[ROZM_TRAN] = "                             ";




    string c1;
    string c2;
    string c3;
    string c4;
    string c5;
    string c6;

    string pwm1;
    string pwm2;
    string dir1;
    string dir2;
    string batt;

    do {

        pthread_mutex_lock(&locker);

        wiringPiSPIDataRW(CHANNEL , TX , 11);

/*
	TX[0]=33;
	TX[1]=22;
	TX[2]=11;
	TX[3]=66;
	TX[4]=55;
	TX[5]=44;
*/


        c1 = to_string(TX[0]);
        c2 = to_string(TX[1]);
        c3 = to_string(TX[2]);
        c4 = to_string(TX[3]);
        c5 = to_string(TX[4]);
        c6 = to_string(TX[5]);

        pwm1 = to_string(TX[6]);
        dir1 = to_string(TX[7]);
        pwm2 = to_string(TX[8]);
        dir2 = to_string(TX[9]);
        batt = to_string(TX[10]);



        strcpy(DANA+0, c1.c_str());
        strcpy(DANA+3, c2.c_str());
        strcpy(DANA+6, c3.c_str());
        strcpy(DANA+9, c4.c_str());
        strcpy(DANA+12, c5.c_str());
        strcpy(DANA+15, c6.c_str());

        strcpy(DANA+18, pwm1.c_str());
        strcpy(DANA+21, dir1.c_str());
        strcpy(DANA+22, pwm2.c_str());
        strcpy(DANA+25, dir2.c_str());

        strcpy(DANA+26, batt.c_str());

      //cout << DANA[0] << DANA[1] << DANA[2] << DANA[3] << DANA[4] << DANA[5] << endl;
      //cout << "cala dana=" << DANA << endl;
      Nadaj(GnNadawania,DANA);

      pthread_mutex_unlock(&locker);

      usleep(100000);
    } while (1);

    close(GnNadawania);

}


















///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////























/*! \brief Klasa odpowiadająca za otrzymywanie danych z innego urządzenia. */

class Receiver{

public:

    /*! Funkcja odpowiadająca za odbieranie danych i przypisanie ich do konkretnych klas.
        \param GnOdbioru - deskryptor gniazda.
     */
    int Odbierz(int GnOdbioru);

    /*! Funkcja aktywująca stan "nasłuchiwania". Gdy dane się pojawią, aktywowana jest funkcja Odbierz, aby przyjąć dane.
        \param Port - zmienna, która odnosi się do otworzonego portu komunikacyjnego.
     */
    int UruchomServer(int Port);


    char DANA[ROZM_REC];
    char info1[11][5];

};



int Receiver::Odbierz(int GnOdbioru){
  ssize_t  IZ;

   while ( IZ = read(GnOdbioru,DANA, ROZM_REC*sizeof(char) ) > 0 ){

      pthread_mutex_lock(&locker);

      for(int i=0 ; i<3 ; i++){
        info1[0][i]=DANA[i];
        info1[2][i]=DANA[i+4];
      }
      // Przyjmowanie danych DIR1, DIR2 i MODE
      info1[1][0]=DANA[3];
      info1[3][0]=DANA[7];
      info1[4][0]=DANA[8];

      TX[0]=atoi(info1[0]);
      TX[1]=atoi(info1[1]);
      TX[2]=atoi(info1[2]);
      TX[3]=atoi(info1[3]);
      TX[4]=atoi(info1[4]);

      pthread_mutex_unlock(&locker);

  }

  return 0;
}


int Receiver::UruchomServer(int Port){
  int  GnNasluchu;
  int  GnPolaczenia;
  struct sockaddr_in    cli_addr, serv_addr;
  socklen_t  ClAddrLen;

  if ((GnNasluchu = socket(AF_INET,SOCK_STREAM,0)) < 0) return -1;

  bzero((char *)&serv_addr,sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  serv_addr.sin_port = htons(Port);
  if (bind(GnNasluchu,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0) {
    return -2;
  }

  ClAddrLen = sizeof(cli_addr);

   if (listen(GnNasluchu,5) < 0)  return -3;

   while (1) {

    GnPolaczenia = accept(GnNasluchu,(struct sockaddr *)&cli_addr,&ClAddrLen);

    if (GnPolaczenia < 0) return -4;
    if (this->Odbierz(GnPolaczenia)) break;

    close(GnPolaczenia);
   }
  close(GnNasluchu);
}




















///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////



















Receiver *receive = new Receiver;
Transmitter *transmit = new Transmitter;
uint8_t buffer[11];

void *Petla_odbiornika(void *){
  receive->UruchomServer(PORT_REC);
  return NULL;
}

void *Petla_nadajnika(void*){
  transmit->SocketConfig(PORT_TRAN , buffer);
  return NULL;
}


int main(){


    wiringPiSetup();
    int fd , result;

    fd = wiringPiSPISetup(CHANNEL , 1000000 );
    if( fd==0 ){
      cerr << "ERROR" << endl;
      return 1;
    }

    pthread_t            Watek_odbiornika;
    pthread_create(&Watek_odbiornika,NULL,Petla_odbiornika,0L);

    cin.get();

    pthread_t            Watek_nadajnika;
    pthread_create(&Watek_nadajnika,NULL,Petla_nadajnika,0L);


    while(1);

    pthread_cancel(Watek_odbiornika);
    pthread_cancel(Watek_nadajnika);

    pthread_join      (Watek_odbiornika,NULL);
    pthread_join      (Watek_nadajnika,NULL);


  return 0;  
}


