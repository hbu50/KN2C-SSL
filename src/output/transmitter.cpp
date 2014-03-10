#include "transmitter.h"

Transmitter::Transmitter(QString port, OutputBuffer* buffer, QObject *parent) :
    QObject(parent),
    _serialport(this),
    _timer(this),
    _buffer(buffer)
{
    qDebug() << "Transmitter Initialization...";

    _serialport.setPortName(port);
    _serialport.setBaudRate(QSerialPort::Baud38400);
    _serialport.open(QIODevice::ReadWrite);
    qDebug() << _serialport.errorString();

    _timer.setInterval(TRANSMITTER_TIMER);
    connect(&_timer,SIGNAL(timeout()), this, SLOT(sendPacket()));
    qDebug() << "Port: " << port;
}

void Transmitter::Start()
{
    _state=true;
    _timer.start();
}

void Transmitter::Stop()
{
    _state=false;
    _timer.stop();
}

void Transmitter::sendPacket()
{
    if(_state)
    {
        QByteArray pck = _buffer->wpck.GetPacket();
        QString log;
        log+= "SerialData[" + QString::number(pck.size()) + "]: ";
        log+= pck.mid(0,2).toHex();
        log+= " ";
        log+= pck.mid(2,1).toHex();
        log+= " ";
        log+= pck.mid(3,1).toHex();
        log+= " ";
        log+= pck.mid(4,1).toHex();
        log+= " ";
        log+= pck.mid(5,2).toHex();
        log+= " ";
        log+= pck.mid(7,2).toHex();
        log+= " ";
        log+= pck.mid(9,2).toHex();
        log+= " ";
        log+= pck.mid(11,2).toHex();
        log+= " ";
        log+= pck.mid(13,1).toHex();
        log+= " ";
        log+= pck.mid(14,1).toHex();
        log+= " ";
        log+= pck.mid(15,1).toHex();
        qDebug() << log;
        _serialport.write(pck);

//        QByteArray debug;
//        debug.append(QByteArray::fromHex("000102030405060708090a0b0c0d0e0f"));
//        log+= "SeialData[" + QString::number(debug.size()) + "]: ";
//        log+= debug.toHex();
//        qDebug() << log;
//        _serialport.port->write(debug);
    }
}
