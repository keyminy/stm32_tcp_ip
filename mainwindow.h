#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPen>
#include <QUdpSocket>
#include <QTextStream>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:

public slots:
    void readyRead();
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

private:
    Ui::MainWindow *ui;
    QUdpSocket *socket = nullptr;
    QVector<double> qv_temp_x,qv_temp_y;
    QVector<double> qv_humid_x, qv_humid_y;
};
#endif // MAINWINDOW_H
