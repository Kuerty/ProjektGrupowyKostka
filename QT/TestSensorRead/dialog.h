#ifndef DIALOG_H
#define DIALOG_H
#include <QSerialPort>
#include <QDialog>
#include <QUdpSocket>

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

private slots:
    void readSerial();

    //void updateLCD(const double, const double , const double, const double , const double , const double );
    void Control_Cursor_by_gyro(double x, double y);

    void Resolution(int &horizontal, int &vertical);

    void on_pushButton_clicked();

    void on_radioButton_clicked();

private:
    Ui::Dialog *ui;

    QSerialPort *arduino;
    static const quint16 arduino_pro_mini_vendor_id = 1027;
    static const quint16 arduino_pro_mini_product_id = 24577;
    QString arduino_port_name;
    bool arduino_is_available;
    QByteArray serialData;
    QUdpSocket *udpsocket;
    QString serialBuffer;
    int horizontal;
    int vertical;
    double accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
    double pitch, roll,compass;
    double quat [4];
    void SaveToFile(QString fname, QString data);


public:
    void send2Scratch(QString data, QString IP_ADD);
};

#endif // DIALOG_H
