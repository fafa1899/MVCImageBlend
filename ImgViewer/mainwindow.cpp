#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_action_triggered()
{
    const char* imagePath = "D:/Work/MVCImageBlend/data/source.jpg";
    ui->imageShowWidget->LoadImage(imagePath);
}

void MainWindow::on_actionBlend_triggered()
{
    const char* imagePath = "D:/Work/MVCImageBlend/data/target.jpg";

    int posX = 370;
    int posY = 290;
    ui->imageShowWidget->ImageBlend(imagePath, posX, posY);
}

void MainWindow::on_actiondraw_triggered(bool checked)
{
    ui->imageShowWidget->SetDraw(checked);
}
