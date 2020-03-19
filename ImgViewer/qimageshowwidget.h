#ifndef QIMAGESHOWWIDGET_H
#define QIMAGESHOWWIDGET_H

#include <QWidget>
#include <opencv2\opencv.hpp>
#include <vector>
#include "geometryalg.h"

class QImageShowWidget : public QWidget
{
    Q_OBJECT
public:
    explicit QImageShowWidget(QWidget *parent = nullptr);
    ~QImageShowWidget();

    bool LoadImage(const char* imagePath);

    void ImageBlend(const char* dstImgPath, int posX, int posY);

    void SetDraw(bool bDraw);

signals:

public slots:

protected:
    void paintEvent(QPaintEvent *);     //绘制
    void mousePressEvent(QMouseEvent *e);       //按下
    void mouseMoveEvent(QMouseEvent *e);        //移动
    void mouseReleaseEvent(QMouseEvent *e);     //松开
    void mouseDoubleClickEvent(QMouseEvent *event);        //双击

    void Release();

    void MVCBlend(int posX, int posY);
    void MVCBlendOptimize(int posX, int posY); //三角网优化

    void CalBoundPoint(std::vector<Vector2d>& ROIBoundPointList);           //栅格化一条线段
    void RasterLine(std::pair<Vector2d, Vector2d> line, std::vector<Vector2d>& linePointList);

    double threePointCalAngle(const Vector2d &P1, const Vector2d &O, const Vector2d &P2);

private:
    uchar* winBuf;      //窗口填充buf
    int winWidth;      //窗口像素宽
    int winHeight;      //窗口像素高
    int winBandNum;      //波段数

    cv::Mat srcImg;
    cv::Mat dstImg;


    bool bDraw;             //是否处于绘制状态
    bool bLeftClick;            //是否已经开始左键点击，同时标识是否开始进行绘制
    bool bMove;             //是否处于绘制时的鼠标移动状态

    QVector<Vector2d> pointList;
    QPointF movePoint;
};

#endif // QIMAGESHOWWIDGET_H
