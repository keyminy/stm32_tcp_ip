#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPen>
#include <QUdpSocket>
#include <QTextStream>
#include <QSerialPort> // UART
#include <QSerialPortInfo> // UART

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:

public slots:
    void readyRead();
    void text_Reading();
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void add_temp_point(double x, double y);
    void add_humid_point(double x,double y);
    void clear_data();
    void plot();
private slots:
    void on_pushButton_send_clicked();

    void on_dial_servo_valueChanged(int value);

    void on_dial_led_valueChanged(int value);

    void on_pushButton_clear_clicked();

    void on_spinBox_DHT11_valueChanged(const QString &arg1);

    void on_pushButton_OpenPort_clicked();

    void on_pushButton_ScanAgain_clicked();

    void on_pushButton_ClosePort_clicked();

    void on_pushButton_Clear_clicked();

    void on_pushButton_SendUart_clicked();

    void on_checkBox_LED1_clicked();

private:
    Ui::MainWindow *ui;
    QUdpSocket *socket = nullptr;
    QVector<double> qv_temp_x,qv_temp_y;
    QVector<double> qv_humid_x, qv_humid_y;
    QSerialPort* serial; // UART port
};
#endif // MAINWINDOW_H
