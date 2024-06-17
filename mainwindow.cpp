#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "qcustomplot.h"
#include <QDebug>
#include <QString>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this); // initialize screen window.
    /* TCP/IP related */
    socket = new QUdpSocket(this);
    // bind socket(-> port & ip)
    bool result = socket->bind(QHostAddress::AnyIPv4,9999);
    qDebug() << result;
    if(result){
        qDebug() << "PASS";
    }else{
        qDebug() << "FAIL";
    }
    // When a SIGNAL comes from the socket, the SLOT function works.
    connect(socket,SIGNAL(readyRead()),this,SLOT(readyRead()));

    ui->plot->setInteraction(QCP::iRangeDrag, true);
    ui->plot->setInteraction(QCP::iRangeZoom, true);

    QPen black_pen,red_pen;
    black_pen.setColor(Qt::black);
    red_pen.setColor(Qt::red);

    // Temperature Graph
    ui->plot->addGraph();
    //ui->plot->xAxis->setLabel("time(s)");
    ui->plot->yAxis->setLabel("temp&hmid");
    ui->plot->yAxis->setRange(15.0, 50.0);

    ui->plot->legend->setVisible(true);
    ui->plot->graph(0)->setName("temperature");
    ui->plot->graph(0)->setScatterStyle(QCPScatterStyle::ssStar);     //점찍는 스타일 결정.
    ui->plot->graph(0)->setLineStyle(QCPGraph::lsLine);                           //라인 스타일 결정.
    ui->plot->graph(0)->setPen(black_pen);

    // Humidity Graph
    ui->plot->addGraph();
    ui->plot->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->plot->graph(1)->setLineStyle(QCPGraph::lsLine);
    ui->plot->graph(1)->setName("Humidity");
    ui->plot->graph(1)->setPen(red_pen);

    // TODO : implement mouse event(not yet)
    connect(ui->plot, SIGNAL(mouseDoubleClickEvent(QMouseEvent*)), SLOT(QMouseEvent*));
    /* END TCP/IP */

    /* ============= RS commuacation UART related===================*/
    //1. set speed
    ui->comboBox_BuadRate->addItem("9600" ,QSerialPort::Baud9600);
    ui->comboBox_BuadRate->addItem("115200" ,QSerialPort::Baud115200);

    ui->comboBox_DataBits->addItem("8",QSerialPort::Data8); //data 8bit
    ui->comboBox_DataBits->addItem("7",QSerialPort::Data7); //data 7bit

    ui->comboBox_ParityBits->addItem("NoParity",QSerialPort::NoParity);
    ui->comboBox_ParityBits->addItem("EvenParity",QSerialPort::EvenParity);
    ui->comboBox_ParityBits->addItem("OddParity",QSerialPort::OddParity);

    // assign port
    serial = new QSerialPort(this);
    // rx INT가 발생하면, textReading()함수 호출한다.
    // serial로부터, rxData=읽을 데이터가 존재하면 -> SLOT함수 textReading()로 간다

    QObject::connect(serial, SIGNAL(readyRead()),this,SLOT(text_Reading()));
    on_pushButton_ScanAgain_clicked();

    /* ============= UART END ===================*/
}

// UART RX INT
void MainWindow::text_Reading(){
    // QByteArray형태의 Object를 할당한다(동적메모리)
    QByteArray received;

    // 읽을 수 있는 만큼 port를 읽음
    received = serial->readLine(); // QbyteArray형태로 집어넣는다
//    while(serial->canReadLine()){
//        // serial쪽에서 "\n"단위(newline)로 들어온다
//        received = serial->readLine(); // QbyteArray형태로 집어넣는다
//    }
     qDebug() << "========ReceivedXXX=== : " <<received << Qt::endl;
    if(received == "") return;
        qDebug() << "Before Received : " <<received;
    received.remove(received.length()-1,2); // remove 끝의 2byte
    this->ui->textEdit_RecvUart->append(received);
    this->ui->textEdit_RecvUart->show(); //창에 나타내준다
    qDebug() << "Received : " <<received << Qt::endl;
}


void MainWindow::readyRead()
{
    double temp_digit,humid_digit;
    QByteArray buffer;
    static int time = 0;

    // resize the data in the socket to its actual size.
    buffer.resize(socket-> pendingDatagramSize());

    QHostAddress sender;
    quint16 senderport;

    // Read the packet in the socket.
    socket->readDatagram(buffer.data()
                         ,buffer.size()
                         ,&sender // ip address
                         ,&senderport // port number
                         );
    // Remove the last 1byte newline character
    buffer.chop(1);
    ui->textEdit_rxdata->append(buffer);
    ui->textEdit_rxdata->show(); // if you don't call show() function, the screen doesn't appear.

    // TODO : Parsing data "[Tmp]26 [Hum]38"
    QString data(buffer);
    QStringList dataList = data.split(' ');

    // Print all elements in dataList
//    qDebug() << "Data List Elements:";
//    for (const QString &element : dataList) {
//        qDebug() << element;
//    }
    /*example
    Data List Elements:
    "[Tmp]26"
    "[Hum]38"
    */
    // Extract temperature value, removing the "[Tmp]" prefix
    QString tempString = dataList.at(0).mid(5);
    // Extract humidity value, removing the "[Hum]" prefix
    QString humidString = dataList.at(1).mid(5);

    // Convert extracted strings to double
     temp_digit = tempString.toDouble();
     humid_digit = humidString.toDouble();

    ui->lcdNumber_temp->display(temp_digit);
    ui->lcdNumber_wet->display(humid_digit);

    //qDebug() << "Message From :: " << sender.toString();
    //qDebug() << "Port from :: " << senderport;
    //qDebug() << "Buffer :: " << buffer;

    add_temp_point(time,temp_digit);
    add_humid_point(time, humid_digit);
    time+=3;
    ui->plot->xAxis->setRange(0, time+3);
    plot();
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_send_clicked()
{
    // input : DHT11_interval:2
    // send data button
    QByteArray sendData;
    sendData = ui->lineEdit_send->text().toUtf8();
    //qDebug() << "sendData : " << sendData << endl;
    socket->writeDatagram(sendData,QHostAddress("10.10.16.120"),9999);

}


void MainWindow::on_dial_servo_valueChanged(int value)
{
    QByteArray servo_data = "SERVO:";
    servo_data.append(QString::number(ui->dial_servo->value()));
    ui->lcdNumber_servo->display(ui->dial_servo->value());

    qDebug() << "servo data : " << servo_data << endl;
    socket->writeDatagram(servo_data,QHostAddress("10.10.16.120"),9999);
}


void MainWindow::on_dial_led_valueChanged(int value)
{
    QByteArray led_data = "LED:";
    led_data.append(QString::number(ui->dial_led->value()));
    ui->lcdNumber_led->display(ui->dial_led->value());

    qDebug() << "led data : " << led_data << endl;
    socket->writeDatagram(led_data,QHostAddress("10.10.16.120"),9999);
}

// add to the vector object(to draw dynamically)
void MainWindow::add_temp_point(double x, double y){
    qv_temp_x.append(x);
    qv_temp_y.append(y);
}
void MainWindow::add_humid_point(double x, double y){
    qv_humid_x.append(x);
    qv_humid_y.append(y);
}

void MainWindow::clear_data(){
    qv_temp_x.clear();
    qv_temp_y.clear();
    qv_humid_x.clear();
    qv_humid_y.clear();
}
// draw graph
void MainWindow::plot(){
    ui->plot->graph(0)->setData(qv_temp_x, qv_temp_y);
    ui->plot->graph(1)->setData(qv_humid_x, qv_humid_y);
    ui->plot->replot();
    ui->plot->update();
}

void MainWindow::on_pushButton_clear_clicked()
{
    clear_data();
}

void MainWindow::on_spinBox_DHT11_valueChanged(const QString &value)
{
    //qDebug() << "value : " << value << endl; // "1"

    QByteArray DHT11_interval_data = "DHT11:";

    DHT11_interval_data.append(QString::number(value.toInt(),10));

    socket->writeDatagram(DHT11_interval_data ,QHostAddress("10.10.16.120"),9999);
    qDebug() << "DHT11 interval data: " << DHT11_interval_data << endl;
    // example : "DHT11_interval:3"
}

void MainWindow::on_pushButton_OpenPort_clicked()
{
    // port이름을 선택할 수 있어야함
    serial->setPortName(ui->comboBox_Device->currentText());
    if(ui->comboBox_BuadRate->currentIndex()==0)
        serial->setBaudRate(QSerialPort::Baud9600);
    else if(ui->comboBox_BuadRate->currentIndex()==1)
        serial->setBaudRate(QSerialPort::Baud115200);

    if(ui->comboBox_DataBits->currentIndex()==0)
        serial->setDataBits(QSerialPort::Data8);
   if(ui->comboBox_DataBits->currentIndex()==1)
        serial->setDataBits(QSerialPort::Data7);

    if(ui->comboBox_ParityBits->currentIndex()==0)
        serial->setParity(QSerialPort::NoParity);
    else if(ui->comboBox_ParityBits->currentIndex()==1)
        serial->setParity(QSerialPort::EvenParity);
    else if(ui->comboBox_ParityBits->currentIndex()==2)
        serial->setParity(QSerialPort::OddParity);

    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    // 설정 다 끝나면 port open
    if(serial->open(QIODevice::ReadWrite)){
        ui->progressBar->setValue(100);
        qDebug() << "\n Serial Port Open Success!!!\n";
    }else{
        qDebug() << "Can not Open Serial Port!!!!!\n";
    }
}

void MainWindow::on_pushButton_ScanAgain_clicked()
{
    //현재 combox box를 지우는 작업
    this->ui->comboBox_Device->clear();

    foreach(const QSerialPortInfo &serialPortInfo,QSerialPortInfo::availablePorts()){
        // 사용가능한 port들을 comboBox에 append해준다
        ui->comboBox_Device->addItem(serialPortInfo.portName());
    }
}


void MainWindow::on_pushButton_ClosePort_clicked()
{
    ui->progressBar->setValue(0);
    serial->close(); //연결 끊기
}


void MainWindow::on_pushButton_Clear_clicked()
{
    ui->textEdit_RecvUart->setText("");
}


void MainWindow::on_pushButton_SendUart_clicked()
{
    //QByteArray형태의 데이터를 port에다가 write하기
    QByteArray send_data;
    send_data = (ui->lineEdit_SendUart->text() + '\n').toUtf8();
    serial->write(send_data.data());
    qDebug() << send_data.data();
}


void MainWindow::on_checkBox_LED1_clicked()
{
    ui->progressBar_LED1->setValue(100);
    QByteArray send_data;
    send_data = ("LED01ON\n"); //UTF8 변환 불필요
    serial->write(send_data.data());
    qDebug() << send_data.data();
}

