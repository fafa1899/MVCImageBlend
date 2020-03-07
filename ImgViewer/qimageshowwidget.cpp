#include "qimageshowwidget.h"

#include <QPainter>
#include <QDebug>
#include <QMouseEvent>
#include <QTime>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

QImageShowWidget::QImageShowWidget(QWidget *parent) : QWidget(parent)
{
    //填充背景色
    setAutoFillBackground(true);
    setBackgroundRole(QPalette::Base);

    winBuf = nullptr;
    winWidth = rect().width();
    winHeight = rect().height();
    winBandNum = 3;

    bDraw = false;
    bLeftClick = false;
    bMove = false;
    pointList.clear();
    setMouseTracking(true);
}

QImageShowWidget::~QImageShowWidget()
{
    if(winBuf)
    {
        delete[] winBuf;
        winBuf = nullptr;
    }
}

bool QImageShowWidget::LoadImage(const char* imagePath)
{
    //从文件中读取成灰度图像
    srcImg = imread(imagePath);
    if (srcImg.empty())
    {
        fprintf(stderr, "Can not load image %s\n", imagePath);
        return false;
    }

    Release();

    winWidth = rect().width();
    winHeight = rect().height();
    size_t winBufNum = (size_t) winWidth * winHeight * winBandNum;
    winBuf = new uchar[winBufNum];
    memset(winBuf, 255, winBufNum*sizeof(uchar));

    for (int ri = 0; ri < srcImg.rows; ++ri)
    {
        for (int ci = 0; ci < srcImg.cols; ++ci)
        {
            for(int bi = 0; bi < winBandNum; bi++)
            {
                size_t m = (size_t) winWidth * winBandNum * ri + winBandNum * ci + bi;
                size_t n = (size_t) srcImg.cols * winBandNum * ri + winBandNum * ci + (winBandNum - 1 - bi);
                winBuf[m] = srcImg.data[n];
            }
        }
    }

    update();

    return true;
}

void QImageShowWidget::ImageBlend(const char* dstImgPath, int posX, int posY)
{
    dstImg = imread(dstImgPath);
    if (dstImg.empty())
    {
        fprintf(stderr, "Can not load image %s\n", dstImgPath);
        return;
    }

    MVCBlend(posX, posY);

    //imwrite("D:\\dst.jpg", dstImg);

    Release();

    winWidth = rect().width();
    winHeight = rect().height();
    size_t winBufNum = (size_t) winWidth * winHeight * winBandNum;
    winBuf = new uchar[winBufNum];
    memset(winBuf, 255, winBufNum*sizeof(uchar));

    for (int ri = 0; ri < dstImg.rows; ++ri)
    {
        for (int ci = 0; ci < dstImg.cols; ++ci)
        {
            for(int bi = 0; bi < winBandNum; bi++)
            {
                size_t m = (size_t) winWidth * winBandNum * ri + winBandNum * ci + bi;
                size_t n = (size_t) dstImg.cols * winBandNum * ri + winBandNum * ci + (winBandNum - 1 - bi);
                winBuf[m] = dstImg.data[n];
            }
        }
    }

    bDraw = false;
    update();
}

void QImageShowWidget::MVCBlend(int posX, int posY)
{
    QTime startTime = QTime::currentTime();

    //Step1:找到边界上所有的像素点
    vector<Vector2d> ROIBoundPointList;
    CalBoundPoint(ROIBoundPointList);

    //Step2:计算范围内每个点的 mean-value coordinates
    size_t srcImgBufNum = static_cast<size_t>(srcImg.cols) * static_cast<size_t>(srcImg.rows);
    vector<vector<double>> MVC(srcImgBufNum);
    for(size_t i = 0; i < srcImgBufNum; i++)
    {
        MVC[i].resize(ROIBoundPointList.size()-1, 0);
    }
    vector<bool> clipMap(srcImgBufNum, true);           //标识范围内的点

    cout<<"开始计算 mean-value coordinates..." << endl;
    #pragma omp parallel for        //开启OpenMP并行加速
    for (int ri = 0; ri < srcImg.rows; ++ri)
    {
        for (int ci = 0; ci < srcImg.cols; ++ci)
        {
            //点是否在多边形内
            size_t m = static_cast<size_t>(srcImg.cols) * ri + ci;
            if(!Point_In_Polygon_2D(ci, ri, ROIBoundPointList))
            {
                clipMap[m] = false;
                continue;
            }

            //逐点计算MVC
            Vector2d P(ci, ri);
            vector<double> alphaAngle(ROIBoundPointList.size());
            for(size_t pi = 1; pi < ROIBoundPointList.size(); pi++)
            {
                alphaAngle[pi] = threePointCalAngle(ROIBoundPointList[pi-1], P, ROIBoundPointList[pi]);
            }
            alphaAngle[0] = alphaAngle[ROIBoundPointList.size()-1];


            for(size_t pi = 1; pi < ROIBoundPointList.size(); pi++)
            {
                double w_a = tan(alphaAngle[pi-1]/2) + tan(alphaAngle[pi]/2);
                double w_b = (ROIBoundPointList[pi-1] - P).Mod();
                MVC[m][pi-1] = w_a / w_b;
                if(_isnan(MVC[m][pi-1])==1)
                {
                    MVC[m][pi-1] = 0;
                }
            }

            double sum = 0;
            for(size_t pi = 0; pi < MVC[m].size(); pi++)
            {
                sum = sum + MVC[m][pi];
            }

            for(size_t pi = 0; pi < MVC[m].size(); pi++)
            {
                MVC[m][pi] = MVC[m][pi] / sum;
            }
        }
    }
    cout<<"计算完成！" << endl;

    //Step3:计算边界的像素插值
    vector<int> diff;
    for(size_t i = 0; i < ROIBoundPointList.size()-1; i++)
    {
        size_t l = (size_t) srcImg.cols * ROIBoundPointList[i].y + ROIBoundPointList[i].x;
        for(int bi = 0; bi < winBandNum; bi++)
        {
            size_t m = (size_t) dstImg.cols * winBandNum * (ROIBoundPointList[i].y + posY)+ winBandNum * (ROIBoundPointList[i].x + posX) + bi;
            size_t n = (size_t) srcImg.cols * winBandNum * ROIBoundPointList[i].y + winBandNum * ROIBoundPointList[i].x + bi;
            int d = (int)(dstImg.data[m]) - (int)(srcImg.data[n]);
            diff.push_back(d);
        }
        clipMap[l] = false;         //在多边形边上的点没法计算MVC
    }

    //Step4:插值计算
    cout<<"开始插值计算..." << endl;
    //Mat rMat(srcImg.rows, srcImg.cols, CV_64FC3);
    #pragma omp parallel for
    for (int ri = 0; ri < srcImg.rows; ++ri)
    {
        for (int ci = 0; ci < srcImg.cols; ++ci)
        {
            size_t l = (size_t) srcImg.cols * ri + ci;
            if(!clipMap[l])
            {
                continue;
            }

            vector<double> r(winBandNum, 0);

            for(size_t pi = 0; pi < MVC[l].size(); pi++)
            {
                for(int bi = 0; bi < winBandNum; bi++)
                {
                    r[bi] = r[bi] + MVC[l][pi] * diff[pi * winBandNum + bi];
                }
            }

            for(int bi = 0; bi < winBandNum; bi++)
            {
                size_t n = (size_t) srcImg.cols * winBandNum * ri + winBandNum * ci + bi;
                size_t m = (size_t) dstImg.cols * winBandNum * (ri + posY)+ winBandNum * (ci + posX) + bi;

                dstImg.data[m] = min(max(srcImg.data[n] + r[bi], 0.0), 255.0);
            }
        }
    }
    cout<<"插值完成！" << endl;

    QTime stopTime = QTime::currentTime();
    int elapsed = startTime.msecsTo(stopTime);
    cout<<"总结完成用时"<<elapsed<<"毫秒";
}

void QImageShowWidget::CalBoundPoint(std::vector<Vector2d>& ROIBoundPointList)
{
    vector<pair<Vector2d, Vector2d>> lineList;
    for(int i = 0; i<pointList.size()-1; i++)
    {
        lineList.push_back(make_pair(pointList[i], pointList[i+1]));
    }

    //遍历所有边，并栅格化
    vector<Vector2d> tmpPointList;
    for(size_t i = 0; i < lineList.size(); i++)
    {
        std::vector<Vector2d> linePointList;
        RasterLine(lineList[i], linePointList);
        std::copy(linePointList.begin(), linePointList.end(), std::back_inserter(tmpPointList));
    }

    ROIBoundPointList.clear();
    ROIBoundPointList.push_back(pointList[0]);
    for(size_t i = 0; i< tmpPointList.size(); i++)
    {
        //与最后一个值比较，去重
        if(!tmpPointList[i].Equel(ROIBoundPointList[ROIBoundPointList.size()-1]))
        {
            ROIBoundPointList.push_back(tmpPointList[i]);
        }
    }
    if(!ROIBoundPointList[0].Equel(ROIBoundPointList[ROIBoundPointList.size()-1]))
    {
        ROIBoundPointList.push_back(ROIBoundPointList[0]);
    }
}

//栅格化一条线段
void QImageShowWidget::RasterLine(std::pair<Vector2d, Vector2d> line, std::vector<Vector2d>& linePointList)
{
    Vector2d vecLine = line.second-line.first;
    double lineLength = vecLine.Mod();
    double step = 1.0;

    vector<Vector2d> tmpPointList;
    double curLength = 0;
    while(curLength<lineLength)
    {
        curLength = curLength + step;
        Vector2d P = line.first + vecLine.Scalar(curLength/lineLength);
        P.x = (int)(P.x + 0.5);
        P.y = (int)(P.y + 0.5);
        tmpPointList.push_back(P);
    }

    //保存起点，不保存终点
    linePointList.push_back(line.first);
    for(size_t i = 0; i< tmpPointList.size(); i++)
    {
        //与最后一个值比较，去重
        if(!tmpPointList[i].Equel(linePointList[linePointList.size()-1]))
        {
            linePointList.push_back(tmpPointList[i]);
        }
    }
}

double QImageShowWidget::threePointCalAngle(const Vector2d &P1, const Vector2d &O, const Vector2d &P2)
{
    Vector2d OP1 = P1 - O;
    Vector2d OP2 = P2 - O;

    double cosAng = OP1.Dot(OP2) / OP1.Mod() / OP2.Mod();
    double ang = acos(cosAng);
    if( _isnan(ang)==1)
    {
        ang=0;
    }

    return ang;
}

void QImageShowWidget::SetDraw(bool bDraw)
{
    this->bDraw = bDraw;
    pointList.clear();
}

//重新实现paintEvent
void QImageShowWidget::paintEvent(QPaintEvent *)
{
    if(!winBuf)
    {
        return;
    }

    QImage::Format imgFomat = QImage::Format_RGB888;

    QPainter painter(this);
    QImage qImg(winBuf, winWidth, winHeight, winWidth*winBandNum, imgFomat);
    painter.drawPixmap(0, 0, QPixmap::fromImage(qImg));

    if(bDraw)
    {
       painter.setPen(QColor(255,0,0));
       QVector<QLineF> lines;
       for(int i = 0; i<pointList.size()-1; i++)
       {
           QLineF line(QPointF(pointList[i].x, pointList[i].y), QPointF(pointList[i+1].x, pointList[i+1].y));
           lines.push_back(line);
       }
       if(bMove&&pointList.size()>0)
       {
           QLineF line(QPointF(pointList[pointList.size()-1].x, pointList[pointList.size()-1].y), movePoint);
           lines.push_back(line);
       }
       painter.drawLines(lines);
    }
}

//按下
void QImageShowWidget::mousePressEvent(QMouseEvent *e)
{
    if(bDraw)
    {
        if(!bLeftClick)
        {
            pointList.clear();
            bLeftClick = true;
        }
    }
    //qDebug()<<"Press";
}

//移动
void QImageShowWidget::mouseMoveEvent(QMouseEvent *e)
{
    if(bDraw&&bLeftClick)
    {
        movePoint = e->pos();
        bMove = true;
        this->update();
    }
    //qDebug()<<"Move";
}

//松开
void QImageShowWidget::mouseReleaseEvent(QMouseEvent *e)
{
    if(bDraw&&bLeftClick)
    {
        pointList.push_back(Vector2d(e->x(), e->y()));
        bMove = false;
        this->update();
    }
    //qDebug()<<"Release";
}

//双击
void QImageShowWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    if(bDraw)
    {
        bLeftClick = false;
        pointList.push_back(pointList[0]);
        this->update();
        //singalDrawOver();
    }
    //qDebug()<<"DoubleClick";
}

void QImageShowWidget::Release()
{
    if(winBuf)
    {
        delete[] winBuf;
        winBuf = nullptr;
    }
}
