#include <QApplication>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QMessageBox>
#include <QDebug>
QSerialPort *arduino;
static const quint16 arduino_uno_vendor_id = 9025;
static const quint16 arduino_MEGA = 66;
QString arduino_port_name;
bool arduino_is_available;

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
        if(serialPortInfo.hasVendorIdentifier()){
            qDebug() << "Port: " << serialPortInfo.portName();
            qDebug() << "Vendor ID: " << serialPortInfo.vendorIdentifier();
        }
        qDebug() << "Has Product ID: " << serialPortInfo.hasProductIdentifier();
        if(serialPortInfo.hasProductIdentifier()){
            qDebug() << "Product ID: " << serialPortInfo.productIdentifier();
         }
    }

    foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
        if(serialPortInfo.hasVendorIdentifier() && serialPortInfo.hasProductIdentifier()){
           if(serialPortInfo.vendorIdentifier() == arduino_uno_vendor_id){
             if(serialPortInfo.productIdentifier() == arduino_MEGA){
                arduino_port_name=serialPortInfo.portName();
                arduino_is_available = true;
             }
           }
        }
    }

    if(arduino_is_available){
        // open and configure the serialport
        arduino->setPortName(arduino_port_name);
        arduino->open(QIODevice::ReadWrite);
        arduino->setBaudRate(QSerialPort::Baud9600);
        arduino->setDataBits(QSerialPort::Data8);
        arduino->setParity(QSerialPort::NoParity);
        arduino->setStopBits(QSerialPort::OneStop);
        arduino->setFlowControl(QSerialPort::NoFlowControl);
    }

    while (arduino->waitForReadyRead(1000)){       /* Condition */
         // arduino->write("\n");
        if(arduino->isReadable()){
                 //qDebug() <<"Readable :"<<arduino->isReadable();
                 QByteArray datoLeido = arduino->readLine();
                 qDebug() <<"Dato :"<< datoLeido;
        }
    }
}
