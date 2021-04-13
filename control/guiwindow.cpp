
#include "guiwindow.h"
#include <QPainter>
#include <math.h>
GuiWindow::GuiWindow(QWidget *parent)
    : QWidget(parent)
{
    x=0;
    y=0;
    image = new QImage(initialWidth, initialHeight, QImage::Format_RGB32);
    image->fill(QColor(100,100,100));
    //offscreenPainter = new QPainter(image);

}

GuiWindow* GuiWindow::guiWindow =0;

void GuiWindow::paintEvent(QPaintEvent */*e*/) {

    centerY= height()/2;
    centerX= width()/2;
    //Q_UNUSED(e);

    QPainter qp(this);
    drawLines(&qp);
    //printf(" paint x: %d y: %d \n ",x, y);
    //zīmē apli
    QPen pen(Qt::blue, 1, Qt::SolidLine);
    qp.setPen(pen);
    qp.drawEllipse(QPoint(centerX,centerY), static_cast<int>(distance*zoom), static_cast<int>(distance*zoom));
    QPen penRed(Qt::red, 3, Qt::SolidLine);
    qp.setPen(penRed);
    qp.drawEllipse(QPoint(centerX+previousX*zoom,centerY-previousY*zoom), 3, 3);
    qp.drawLine(centerX+previousX*zoom,centerY-previousY*zoom,centerX+previousX*zoom+10*cos(alfaGt),centerY-previousY*zoom -10*sin(alfaGt));

    QPen penGreen(Qt::green, 1, Qt::SolidLine);
    qp.setPen(penGreen);
    qp.drawEllipse(QPoint(centerX+previousXEst*zoom,centerY-previousYEst*zoom), 3, 3);

    for(Particle p: particles){
        qp.drawEllipse(QPoint(centerX+p.x*zoom,centerY-p.y*zoom), 3, 3);
        qp.drawLine(centerX+p.x*zoom,centerY-p.y*zoom,centerX+p.x*zoom+10*cos(p.direction),centerY-p.y*zoom -10*sin(p.direction));


    }

    QPen penBlue(Qt::blue, 2, Qt::SolidLine);
    qp.setPen(penBlue);

    qp.drawEllipse(QPoint(centerX+avgParticle.x*zoom,centerY-avgParticle.y*zoom), 3, 3);
    qp.drawLine(centerX+avgParticle.x*zoom,centerY-avgParticle.y*zoom,centerX+avgParticle.x*zoom+10*cos(avgParticle.direction),centerY-avgParticle.y*zoom -10*sin(avgParticle.direction));


    update();
}

void GuiWindow::drawLines(QPainter *qp) {
    //std::set<QPoint*>::iterator it;
    //QPen pen(Qt::black, 2, Qt::SolidLine);
    //qp->setPen(pen);
    QPoint startPoint(0,0);
    qp->drawImage(startPoint,*image );
    //qp->drawPoint(50+(x*zoom),50+(y*zoom));
    // for (it=actualTrajectory.begin(); it!=actualTrajectory.end(); ++it)
    //   qp->drawPoint(50+(it.x()*zoom),50+(it.y()*zoom));

}

QSize GuiWindow::minimumSizeHint() const
{
    return QSize(100, 100);
}

QSize GuiWindow::sizeHint() const
{
    return QSize(initialWidth, initialHeight);
}


void GuiWindow::updateView(float x, float y,double alfa)
{
    /// offscreen drawing
    QPainter p;
    QPen pen(Qt::red, 1, Qt::SolidLine);
    p.begin(image);
    p.setPen(pen);
    p.drawLine(10,20,10+10*zoom,20);
    p.drawText(10,18,"10 m");
    //p.drawPoint(centerX+static_cast<int>(x*zoom),centerY-static_cast<int>(y*zoom));        // drawing code

    if(previousX!=0&&previousY!=0)
        p.drawLine(centerX+static_cast<int>(previousX*zoom),centerY-static_cast<int>(previousY*zoom),centerX+static_cast<int>(x*zoom),centerY-static_cast<int>(y*zoom));
    p.end();
    // actualTrajectory.insert(new QPoint(x,y));
    //  update();
    previousX = x;
    previousY = y;
    alfaGt =alfa;

    //repaint();
    //printf(" lines x: %d y: %d \n ",x, y);

}

void GuiWindow::updateFromMainThread()
{
    update();
}
void GuiWindow::updateEstimateView(float x, float y)
{
    /// offscreen drawing
    QPainter p;
    QPen pen(Qt::green, 1, Qt::SolidLine);
    p.begin(image);
    p.setPen(pen);

    //p.drawPoint(centerX+static_cast<int>(x*zoom),centerY-static_cast<int>(y*zoom));        // drawing code

    if(previousXEst!=0&&previousYEst!=0)
        p.drawLine(centerX+static_cast<int>(previousXEst*zoom),centerY-static_cast<int>(previousYEst*zoom),centerX+static_cast<int>(x*zoom),centerY-static_cast<int>(y*zoom));
    p.end();
    // actualTrajectory.insert(new QPoint(x,y));
    //   update();
    previousXEst = x;
    previousYEst = y;
    //repaint();
    //printf(" lines x: %d y: %d \n ",x, y);

}

void GuiWindow::updateDistanceView(double distance)
{
    this->distance=distance;
}
