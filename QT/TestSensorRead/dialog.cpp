#include "dialog.h"
#include "ui_dialog.h"
#include <QSerialPort>
#include <QSerialPortInfo>
#include <string>
#include <QDebug>
#include <QtWidgets>
#include <windows.h>
#include "wtypes.h"
#include <iostream>
#include <math.h>
#include <QFile>
#include <QIODevice>

POINT P;
int cx,cy;
float g_x,g_y,g_z,a_x,a_y,a_z;


int up_wall, north_wall,south_wall,east_wall,west_wall,bottom_wall,button_state, obrot;
int last_wall_4=1;
int wall [6];
double yaw;
bool interface_mode = 1;

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);

    udpsocket = new QUdpSocket(this);
    arduino = new QSerialPort(this);
    arduino_port_name = "";
    arduino_is_available = false;
    serialBuffer = "";
    QString PORTNAME;
    PORTNAME = "COM5";


    Resolution(horizontal, vertical);


    /*
    qDebug() << "Number of avablive ports: " << QSerialPortInfo::availablePorts().length();
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
        qDebug() << "Has vendor ID: " << serialPortInfo.hasVendorIdentifier();
        if (serialPortInfo.hasVendorIdentifier()){
            qDebug() << "vendor ID: " << serialPortInfo.vendorIdentifier();
        }
        qDebug() << "Has product ID: " << serialPortInfo.hasProductIdentifier();
        if (serialPortInfo.hasProductIdentifier()){
            qDebug() << "Produt ID :" << serialPortInfo.productIdentifier();
        }

    }
    */

    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()) {   //sprawdzenie dla każdego dostępnego portu
        ui->lista_portow_COM->addItem(serialPortInfo.portName());                          //dodaje każdy dostępny port do ręcznej
                                                                                           //listy w GUI
        if (serialPortInfo.hasVendorIdentifier() && serialPortInfo.hasProductIdentifier()){//rozpoczyna automatyczne sprawdzenie
                                                                                           //czy dane urządzenie dostępne na
                                                                                           //na porcie jest kostką
            if(serialPortInfo.vendorIdentifier() == arduino_pro_mini_vendor_id){           //prawda jeżeli zgadza się vendor id
                if (serialPortInfo.productIdentifier() == arduino_pro_mini_product_id){    //prawda jeżeli zgadza się product id
                    arduino_port_name = serialPortInfo.portName();                         //zapisz nazwe portu szeregowego
                    arduino_is_available = true;                                           //zapisz informacje że znaleziono kostke
                }
            }
        }
    }

    if (true){
        // open and configure serialport
        arduino->setPortName(PORTNAME);
        arduino->open(QSerialPort::ReadWrite);
        arduino->setBaudRate(QSerialPort::Baud115200);
        arduino->setDataBits(QSerialPort::Data8);
        arduino->setParity(QSerialPort::NoParity);
        arduino->setStopBits(QSerialPort::OneStop);
        arduino->setFlowControl(QSerialPort::NoFlowControl);
        QObject::connect(arduino, SIGNAL(readyRead()), this, SLOT(readSerial()));
    }
    else{
        // give error message
        QMessageBox::warning(this, "Port error", "Couldn't find the Arduino");
    }
}

Dialog::~Dialog()
{
    if(arduino->isOpen()){
        arduino->close();
    }
    delete ui;
}



void Dialog::readSerial(){
    QByteArray ba;
    QString data;
    QStringList data_split;


//    qDebug()<< data;
/*
    if(data.contains("p") && data.contains("k"))
    {
        //int c = data.count("p");
        data.remove("<");
        data.remove(">");

        int ts = data.indexOf("p");
        int te = data.indexOf("k");

        data.remove(0,ts+1);
        data.remove(te-1, data.length());
        data.remove("p");
        data.remove("k");
        if (data.length() > 7){
//      if (data.length() > 29){
            data.remove("\r");
            qDebug()<<"dane od p do k: "<<data;

            data_split = data.split(";");
            pitch = data_split[0].toDouble();
            roll = data_split[1].toDouble();


            accel_x = data_split[0].toDouble();
            accel_y = data_split[1].toDouble();
            accel_z = data_split[2].toDouble();
            gyro_x = data_split[3].toDouble();
            gyro_y = data_split[4].toDouble();
            gyro_z = data_split[5].toDouble();

            Dialog::updateLCD(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
            QString message2Scratch = trUtf8("sensor-update Signal1 %1 Signal2 %2 Signal3 %3 Signal4 %4 Signal5 %5 Signal6 %6")
                    .arg(accel_x).arg(accel_y).arg(accel_z).arg(gyro_x).arg(gyro_y).arg(gyro_z);
            QString message2Scratch2 = trUtf8 ("sensor-update Signal1 %1 Signal2 %2").arg(pitch).arg(roll);
//          send2Scratch(message2Scratch, "127.0.0.1");
            send2Scratch(message2Scratch2, "127.0.0.1");

        }
    }
*/


   //qDebug() << serialBuffer;
/*
    serialData = arduino->readAll();
    serialBuffer += QString::fromStdString(serialData.toStdString());
    qDebug() << serialBuffer;
*/



    while (arduino->canReadLine()){
       ba = arduino->readLine();
       data=ba;

       SaveToFile("C:\\D\\dataforcalibration\\dane1.txt",data);
       //if(QFile datafile("C:\\D\\dataforcalibration\\data1.txt", QIODevice::WriteOnly((
       //datafile.open(QIODevice::WriteOnly);

       /*
       QString message2Scratch2;
       //qDebug() << data;
       int last_up_wall;
       int last_north_wall;

       south_wall = 7-north_wall;
       bottom_wall = 7-up_wall;
       if(up_wall == 6){
           if(north_wall == 2)
               east_wall = 3;
           west_wall = 7-east_wall;
       }

       bool dosplit = false;

       if(dosplit)
       {
       data_split = data.split(";");

       wall[5] = data_split[0].toInt();
       wall[4] = data_split[1].toInt();
       if (interface_mode == 1){
           wall[3] = data_split[2].toInt();
           wall[2] = data_split[3].toInt();
           wall[1] = data_split[4].toInt();
           wall[0] = data_split[5].toInt();
           up_wall = data_split[6].toInt();
           north_wall = data_split[7].toInt();
       }
       else{
           roll = data_split[2].toDouble();
           yaw = data_split[3].toDouble();
           pitch = data_split[4].toDouble();
       }

       south_wall = 7-north_wall;
       bottom_wall = 7-up_wall;
       if (interface_mode ==1){
           if (north_wall == 1){    //jeżeli pólnocna ściana to 1
            switch(up_wall){        //i ściana górna:
            case 2:                 //2
                west_wall =3;       //to zachodnia będzie 3
                break;
            case 3:                 //3
                west_wall =5;       //to zachodnia będzie 5 itd...
                break;
            case 4:
                west_wall =2;
                break;
            case 5:
                west_wall=4;
                break;
            }
       }

            if (north_wall == 2){
             switch(up_wall){
             case 1:
                 west_wall =4;
                 break;
             case 3:
                 west_wall =1;
                 break;
             case 4:
                 west_wall =6;
                 break;
             case 6:
                 west_wall=3;
                 break;
             }
            }

             if (north_wall == 3){
              switch(up_wall){
              case 1:
                  west_wall =2;
                  break;
              case 2:
                  west_wall =6;
                  break;
              case 5:
                  west_wall =1;
                  break;
              case 6:
                  west_wall=5;
                  break;
              }
             }

              if (north_wall == 4){
               switch(up_wall){
               case 1:
                   west_wall =5;
                   break;
               case 2:
                   west_wall =1;
                   break;
               case 5:
                   west_wall =6;
                   break;
               case 6:
                   west_wall=2;
                   break;
               }
              }

               if (north_wall == 5){
                switch(up_wall){
                case 1:
                    west_wall =3;
                    break;
                case 3:
                    west_wall =6;
                    break;
                case 4:
                    west_wall =1;
                    break;
                case 6:
                    west_wall=4;
                    break;
                }
               }

                if (north_wall == 6){
                 switch(up_wall){
                 case 2:
                     west_wall =4;
                     break;
                 case 3:
                     west_wall =2;
                     break;
                 case 4:
                     west_wall =5;
                     break;
                 case 5:
                     west_wall=3;
                     break;
                 }
                }
        */

        /*
       QVector <int> sciany;

       for(int i=1;i<7;i++)
       {
           sciany.push_back(i);
       }


           sciany.remove(north_wall);
           sciany.remove(up_wall);
           sciany.remove(bottom_wall);
           sciany.remove(south_wall);


       for(int i=0;i<sciany.size();i++)
       {
           qDebug()<<sciany[i];
       }

       */

      //      east_wall = 7-west_wall;    //wschodnia 7-wschodnia
      // }
       /*
       quat[0] = data_split[0].toDouble();
       quat[1] = data_split[1].toDouble();
       quat[2] = data_split[2].toDouble();
       quat[3] = data_split[3].toDouble();
       double ysqr = quat [2] * quat [2];
       double t0 = +2.0f * (quat[0] * quat[1] + quat[2] * quat[3]);
       double t1 = +1.0f - 2.0f * (quat[1] * quat[1] + ysqr);
       double t2 = +2.0f * (quat[0] * quat[2] - quat[3] * quat[1]);
       double t3 = +2.0f * (quat[0] * quat[3] + quat[1] *quat[2]);
       double t4 = +1.0f - 2.0f * (ysqr + quat[3] * quat[3]);


       katy = 180/3.14;
       roll = atan2(t0, t1);
       pitch = asin(t2);
       yaw = atan2(t3,t4);
       double roll_s = roll*katy;
       double pitch_s = pitch * katy;
       double yaw_s = yaw * katy;
       qDebug() << pitch_s << " " << roll_s << " " << yaw_s;
       */
       /*
       roll = atan2((2*quat[2]*quat[0]) + (2*quat[1]*quat[2]), 1- (2*quat[2]*quat[2]) - (2*quat[3]*quat[3]) );
       pitch = atan2((2*quat[1]*quat[0]) + (2*quat[2]*quat[3]), 1- (2*quat[1]*quat[1]) - (2*quat[3]*quat[3]) );
       yaw = asin((2*quat[1]*quat[2]) + (2*quat[3]*quat[0]));
       qDebug() << pitch << " " << roll << " " << yaw;
       */


/*
       up_wall = data_split[0].toInt();
       north_wall = data_split[1].toInt();
       button_state = data_split[2].toInt();
       roll = data_split[3].toDouble();
       yaw = data_split[4].toDouble();
       wall[0] = data_split[5].toInt();
       wall[1]= data_split[6].toInt();
       wall[2] = data_split[7].toInt();
       wall[3] = data_split[8].toInt();
       wall[4] = data_split[9].toInt();
       wall[5] = data_split[10].toInt();
*/
/*
       accel_x = data_split[2].toDouble();
       accel_y = data_split[3].toDouble();
       accel_z = data_split[4].toDouble();
       gyro_x = data_split[5].toDouble();
       gyro_y = data_split[6].toDouble();
       gyro_z = data_split[7].toDouble();
*/
       //updateLCD (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
       //qDebug () << data_split;
       //qDebug () << "pitch : " << pitch << " roll : " << roll;



/*
       if (last_wall_4 == 0){
           if(wall [4] = 1)
               if(arduino->isWritable()){
                   interface_mode = !interface_mode;
                   if (interface_mode)
                       arduino->write("0");
                   else
                       arduino->write("1");
               }
               else
                   qDebug() << "can't write";
       }

}
       if(interface_mode == 1){
           message2Scratch2 = trUtf8 ("sensor-update top_wall %1 north_wall %2 wall1 %3 wall2 %4 wall3 %5 wall4 %6 wall5 %7 wall6 %8 obrot %9 bottom_wall %10 south_wall %11 east_wall %12 west_wall %13").arg(up_wall).arg(north_wall).arg(wall[0]).arg(wall[1]).arg(wall[2]).arg(wall[3]).arg(wall[4]).arg(wall[5]) .arg(obrot).arg(bottom_wall).arg(south_wall).arg(east_wall).arg(west_wall);
       }
       else{
           int przycisk = wall[5];
           message2Scratch2 = trUtf8 ("sensor-update yaw %1 roll %2 wall5 %3 pitch %4").arg(-yaw).arg(roll).arg(wall[5]).arg(pitch);
           if(przycisk){
               GetCursorPos( & P );
               cx=P.x;
               cy=P.y;
           }
           if (!przycisk){
               double bok = (((pitch/3)-((2*yaw)/3)));
                Control_Cursor_by_gyro(bok,roll);
           }
       }

       ui->label->setText(data);
       ui->s1->setText(QString::number(up_wall));
       ui->s2->setText(QString::number(west_wall));
       ui->s3->setText(QString::number(south_wall));
       ui->s4->setText(QString::number(north_wall));
       ui->s5->setText(QString::number(east_wall));
       ui->s6->setText(QString::number(bottom_wall));
       send2Scratch(message2Scratch2, "127.0.0.1");
       last_up_wall =up_wall;
       last_north_wall=north_wall;
       last_wall_4 = wall[4];
       //qDebug () << data;
       data.chop(2);
*/
       /*
       QStringList splitdata = data.split("\t");
       qDebug () << splitdata;
       double X = splitdata [0].toDouble();
       double Y = splitdata [1].toDouble();
       double Z = splitdata [2].toDouble();
       X = X/4096;
       Y = Y/4096;
       Z = Z/4096;

       data =X;
       data +="\t";
       data +=Y;
       data +="\t";
       data +=Z;
       */

       ui->textEdit->append(data);
//
    }

}



void Dialog::send2Scratch(QString data, QString IP_ADD)
{
    QByteArray ByteData = data.toLocal8Bit();

    int n = ByteData.length();
    QByteArray BArray;
    BArray.clear();


    BArray.append(char((n >> 24) & 0xFF));
    BArray.append(char((n >> 16) & 0xFF));
    BArray.append(char((n >> 8) & 0xFF));
    BArray.append(char(n & 0xFF));

    BArray.append(ByteData);

    udpsocket->writeDatagram(ByteData, QHostAddress(IP_ADD), 42001);
}

void Dialog::Control_Cursor_by_gyro(double x, double y){
   int cursor_x,cursor_y;
   if (x >=100)
       x=100;
   if (x <= -100)
       x = -100;
   if (y>=80)
       y=80;
   if (y<= -80)
       y=-80;
   cursor_x = (int) ((horizontal*x)/100);
   cursor_y = (int) (-(horizontal*y)/80);
   SetCursorPos(cx + cursor_x, cy + cursor_y);

//   qDebug() << cursor_x << "  " << cursor_y;

}

void Dialog::Resolution(int& horizontal, int& vertical)
{
   RECT desktop;
   // Get a handle to the desktop window
   const HWND hDesktop = GetDesktopWindow();
   // Get the size of screen to the variable desktop
   GetWindowRect(hDesktop, &desktop);
   // The top left corner will have coordinates (0,0)
   // and the bottom right corner will have coordinates
   // (horizontal, vertical)
   horizontal = desktop.right;
   vertical = desktop.bottom;
}


void Dialog::on_pushButton_clicked()
{
    if(arduino->isWritable()){
        interface_mode = !interface_mode;
        if (interface_mode)
            arduino->write("0");
        else
            arduino->write("1");
    }
    else
        qDebug() << "cant write";
}


void Dialog::on_radioButton_clicked()
{

}

void Dialog::SaveToFile(QString fname, QString data)
{
    //QString filename="Data.txt";
    QFile file( fname );
    if ( file.open(QIODevice::ReadWrite | QIODevice::Append) )
    {
        QTextStream stream( &file );
        stream << data << endl;
    }
}
