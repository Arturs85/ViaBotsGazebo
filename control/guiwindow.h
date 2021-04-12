#ifndef GUIWINDOW
#define GUIWINDOW


#pragma once

#include <QWidget>
#include <set>
 #include <QImage>
#include <QPainter>
#include <QPoint>


class GuiWindow : public QWidget
{
    Q_OBJECT


public:
    GuiWindow(QWidget *parent = 0);
   // ~GuiWindow();
    void updateView(float x,float y,double alfa);
    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint() const Q_DECL_OVERRIDE;
    int x;
    int y;
    double alfaGt=0;
    const int zoom = 10;
    const int initialHeight = 800;
    int initialWidth = 1000;
    int centerY= initialHeight/2;
    int centerX= initialWidth/2;

    void updateEstimateView(float x, float y);
    void updateDistanceView(double distance);
protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
    void drawLines(QPainter *qp);
  QImage* image;
  QPainter *offscreenPainter;
  std::set<QPoint*> actualTrajectory;
float previousX=0;
float previousY=0;
float previousXEst=0;
float previousYEst=0;
double distance = 0;
};

#endif // GUIWINDOW
