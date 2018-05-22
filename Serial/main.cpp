#include <QApplication>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
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
int dat1, dat2, theta;

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

    while (arduino->waitForReadyRead(1000)){       /* Condition */
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
                 dat1=data1.toInt();
                 dat2=data2.toInt();
                 theta=(dat1-543)/(dat2-543);
                 qDebug() <<"Dato 1:"<< dat1;
                 qDebug() <<"Dato 2:"<< dat2;
                 qDebug() <<"Angulo:  "<< theta;
                 //serialBuffer="";
                 }
    }
   }
 }

