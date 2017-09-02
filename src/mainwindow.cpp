#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "parallel_threshold.h"



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}



QImage MainWindow::putDepth(const cv::Mat& input)
{
    QImage output;
    cv::Mat mat;

    if (input.type() == CV_16UC1){
        cv::Mat depthf(cv::Size(640,480),CV_8UC1);
        input.convertTo(depthf, CV_8UC1, 255.0/2048.0);
        mat = depthf;

//        cv::Mat bilateralImage;
//        cv_extend::bilateralFilter(depthf,bilateralImage,50,10);
//        cv::imshow("bilateral",bilateralImage);
    } else {
        mat = input;
    }

    // 8-bits unsigned, NO. OF CHANNELS=1
    if(mat.type()==CV_8UC1)
    {
        // Set the color table (used to translate colour indexes to qRgb values)
        QVector<QRgb> colorTable;
        for (int i=0; i<256; i++)
            colorTable.push_back(qRgb(i,i,i));
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        output = img;
    }
    // 8-bits unsigned, NO. OF CHANNELS=3
    else if(mat.type()==CV_8UC3)
    {
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        output = img.rgbSwapped();
    }
    else
    {
        std::cout << "ERROR: Mat could not be converted to QImage." << std::endl;
    }


    int w = ui->depthLabel->width();
    int h = ui->depthLabel->height();

    QPixmap p = QPixmap::fromImage(output);

    // set a scaled pixmap to a w x h window keeping its aspect ratio
    ui->depthLabel->setPixmap(p.scaled(w,h,Qt::KeepAspectRatio));
//    ui->imageLabel->setPixmap(p);


//    ui->bilateralLabel->setPixmap(b.scaled(w,h,Qt::KeepAspectRatio))


    return output;
}


QImage MainWindow::putImage(const cv::Mat& mat)
{
	QImage output;

    // 8-bits unsigned, NO. OF CHANNELS=1
    if(mat.type()==CV_8UC1)
    {
        // Set the color table (used to translate colour indexes to qRgb values)
        QVector<QRgb> colorTable;
        for (int i=0; i<256; i++)
            colorTable.push_back(qRgb(i,i,i));
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        output = img;
    }
    // 8-bits unsigned, NO. OF CHANNELS=3
    if(mat.type()==CV_8UC3)
    {
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        output = img.rgbSwapped();
    }
    else
    {
        std::cout << "ERRORa: Mat could not be converted to QImage." << std::endl;
    }


    int w = ui->imageLabel->width();
    int h = ui->imageLabel->height();

    QPixmap p = QPixmap::fromImage(output);

    // set a scaled pixmap to a w x h window keeping its aspect ratio
    ui->imageLabel->setPixmap(p.scaled(w,h,Qt::KeepAspectRatio));
//    ui->imageLabel->setPixmap(p);


    return output;
}
