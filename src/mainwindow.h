#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "globals.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QImage putImage(const cv::Mat& mat);
    QImage putDepth(const cv::Mat& input);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
