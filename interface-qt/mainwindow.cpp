#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QScrollBar>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    arduino = new QSerialPort(this);


    arduino->setPortName("COM10");
    arduino->setBaudRate(QSerialPort::Baud9600);

    if (!arduino->open(QIODevice::ReadWrite)) {
        qDebug() << "Erro ao abrir porta serial:" << arduino->errorString();
    }

    connect(ui->btnSequencia, &QPushButton::clicked, this, &MainWindow::iniciarSequencia);
    connect(ui->btnParar, &QPushButton::clicked, this, &MainWindow::pararMotor);
    connect(ui->btnDefinirVelocidade, &QPushButton::clicked, this, &MainWindow::definirVelocidade);
    connect(ui->btnNorte, &QPushButton::clicked, this, &MainWindow::buscarNorte);
    connect(ui->btnNivelar, &QPushButton::clicked, this, &MainWindow::nivelar_pitch);
    connect(ui->btnInicializacao, &QPushButton::clicked, this, &MainWindow::inicializacao);

    connect(ui->btnAlvo, &QPushButton::clicked, this, &MainWindow::definirAlvo);


    connect(arduino, &QSerialPort::readyRead, this, &MainWindow::lerDadosSerial);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::iniciarSequencia()
{
    arduino->write("S\n");
}

void MainWindow::pararMotor()
{
    arduino->write("P\n");
}

void MainWindow::definirVelocidade()
{
    QString rpm = ui->txtVelocidade->text();
    QString comando = "V" + rpm + "\n";
    arduino->write(comando.toUtf8());
}

void MainWindow::lerDadosSerial()
{
    QByteArray dados = arduino->readAll();
    ui->txtLog->insertPlainText(QString::fromUtf8(dados));

    QScrollBar *bar = ui->txtLog->verticalScrollBar();
    bar->setValue(bar->maximum());
}
void MainWindow::buscarNorte()
{
    arduino ->write("N\n");
}

void MainWindow::nivelar_pitch()
{
    arduino ->write("I\n");
}
void MainWindow::inicializacao()
{
    arduino ->write("CALIBRAR\n");
}
void MainWindow::definirAlvo()
{
    QString Di = ui->txtDirecInicial->toPlainText().trimmed();
    QString Df = ui->txtDirecFinal->toPlainText().trimmed();
    QString Ei = ui->txtElevInicial->toPlainText().trimmed();
    QString Ef = ui->txtElevFinal->toPlainText().trimmed();
    QString Em = ui->txtElevMax->toPlainText().trimmed();
    QString tempo = ui->txtTemp->toPlainText().trimmed();

    QString pacote = "Alvo:" + Di + "," + Df + "," + Ei + "," + Ef + "," + Em + "," + tempo + "\n";
    arduino->write(pacote.toUtf8());
}

