#include <QApplication>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>

#include <valarray>
#include <unistd.h>

QSerialPort *arduino;
static const quint16 arduino_uno_vendor_id = 9025;
static const quint16 arduino_MEGA = 66;
QString arduino_port_name;
bool arduino_is_available;
QByteArray dataread;
QString serialBuffer;
QString serialBuffer1;
QString data1;
QString data2;
float x, y, relx, rely, theta, psi;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    arduino_is_available = false;
    arduino_port_name = "";
    arduino = new QSerialPort;


    //Parte # 2,buscar puertos con los identificadores de Arduino
    qDebug() << "Number of available ports: " << QSerialPortInfo::availablePorts().length();
    foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
        qDebug() << "Has vendor ID: " << serialPortInfo.hasVendorIdentifier();
        if(serialPortInfo.hasVendorIdentifier() && serialPortInfo.portName()=="ttyACM0"){
            qDebug() << "Port: " << serialPortInfo.portName();
            qDebug() <<"\n";
            qDebug() << "Vendor ID: " << serialPortInfo.vendorIdentifier();
            qDebug() << "Has Product ID: " << serialPortInfo.hasProductIdentifier();
            qDebug() << "Product ID: " << serialPortInfo.productIdentifier();
         }
    }


    foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
        if(serialPortInfo.hasVendorIdentifier() && serialPortInfo.hasProductIdentifier()){
           if(serialPortInfo.vendorIdentifier() == arduino_uno_vendor_id){
             if(serialPortInfo.productIdentifier() == arduino_MEGA){
               if(serialPortInfo.portName()=="ttyACM0") {
                arduino_port_name=serialPortInfo.portName();
                arduino_is_available = true;
               }
             }
           }
        }
     }

    if(arduino_is_available && arduino_port_name=="ttyACM0"){
        // open and configure the serialport
        arduino->setPortName(arduino_port_name);
        arduino->open(QIODevice::ReadWrite);
        arduino->setBaudRate(QSerialPort::Baud9600);
        arduino->setDataBits(QSerialPort::Data8);
        arduino->setParity(QSerialPort::NoParity);
        arduino->setStopBits(QSerialPort::OneStop);
        arduino->setFlowControl(QSerialPort::NoFlowControl);
        //QObject::connect(arduino, SIGNAL(readyRead()), this, SLOT(readSerial()));
    }
std::valarray<double> meanp(100),meant(100);
    while (arduino->waitForReadyRead(1000)){       /* Condition */

        for (uint i=0; i<meanp.size(); i++)
        {
        if(arduino->isReadable()){
                 dataread = arduino->readLine();
                 if(dataread!="\n"){
                 qDebug() <<"Dato total:"<<dataread;
                 serialBuffer = QString::fromStdString(dataread.data());
                 serialBuffer1 = QString::fromStdString(dataread.data());
                 //split
                 data2=serialBuffer1.remove(3,4);
                 data1=serialBuffer.remove(0,3);
                 data1=data1.remove(3,1);
                 //covert to int
                 y=data1.toFloat();
                 x=data2.toFloat();
                 rely=(y-541)*90/168;
                 relx=(x-541)*90/168;
                 //Angle
                 if ((abs(relx)<1000) & (abs(rely)<1000))
                 {
                 psi=atan(relx/rely);
                 theta=(relx*sin(psi)+rely*sin((M_PI/2)-psi))/100;
                 }
                 else
                 {
                     qDebug() <<""
                                "Error psi:  "<< relx;
                     qDebug() <<"Error theta:  "<< rely;
                 }
                 //theta=atan((relx*sin(psi))/(rely*sin((M_PI/2)-psi)));

                 meanp[i]=psi;
                 meant[i]=theta;

                 }
        }
        qDebug() <<"y  :"<< rely;
        qDebug() <<"x  :"<< relx;
//        qDebug() <<"Angulo psi:  "<< (meanp.sum()/meanp.size())*180/M_PI;
//        qDebug() <<"Angulo theta:  "<< (meant.sum()/meant.size())*180/M_PI;
        qDebug() <<"Angulo psi:  "<< psi*180/M_PI;
        qDebug() <<"Angulo theta:  "<< theta*180/M_PI;
        }
        usleep(100000);

    }
}

