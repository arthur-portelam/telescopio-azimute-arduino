#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void iniciarSequencia();
    void pararMotor();
    void definirVelocidade();
    void lerDadosSerial();
    void buscarNorte();
    void nivelar_pitch();
    void inicializacao();
    void definirAlvo();


private:
    Ui::MainWindow *ui;
    QSerialPort *arduino;
};

#endif // MAINWINDOW_H