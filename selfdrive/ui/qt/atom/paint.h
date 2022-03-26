#pragma once

#include <QStackedLayout>
#include <QWidget>


#include "selfdrive/ui/ui.h"



class OnPaint : public QWidget 
{
  Q_OBJECT

  Q_PROPERTY(bool showVTC MEMBER showVTC NOTIFY valueChanged);
  Q_PROPERTY(QString vtcSpeed MEMBER vtcSpeed NOTIFY valueChanged);
  Q_PROPERTY(QColor vtcColor MEMBER vtcColor NOTIFY valueChanged);
  Q_PROPERTY(bool showDebugUI MEMBER showDebugUI NOTIFY valueChanged);

  Q_PROPERTY(QString roadName MEMBER roadName NOTIFY valueChanged);

  Q_PROPERTY(bool showSpeedLimit MEMBER showSpeedLimit NOTIFY valueChanged);
  Q_PROPERTY(QString speedLimit MEMBER speedLimit NOTIFY valueChanged);
  Q_PROPERTY(QString slcSubText MEMBER slcSubText NOTIFY valueChanged);
  Q_PROPERTY(float slcSubTextSize MEMBER slcSubTextSize NOTIFY valueChanged);
  Q_PROPERTY(bool mapSourcedSpeedLimit MEMBER mapSourcedSpeedLimit NOTIFY valueChanged);
  Q_PROPERTY(bool slcActive MEMBER slcActive NOTIFY valueChanged);

  Q_PROPERTY(bool showTurnSpeedLimit MEMBER showTurnSpeedLimit NOTIFY valueChanged);
  Q_PROPERTY(QString turnSpeedLimit MEMBER turnSpeedLimit NOTIFY valueChanged);
  Q_PROPERTY(QString tscSubText MEMBER tscSubText NOTIFY valueChanged);
  Q_PROPERTY(bool tscActive MEMBER tscActive NOTIFY valueChanged);
  Q_PROPERTY(int curveSign MEMBER curveSign NOTIFY valueChanged);


public:
  explicit OnPaint(QWidget *parent);
  void updateState(const UIState &s);

private:
  void    paintEvent(QPaintEvent *event) override;
  void    mousePressEvent(QMouseEvent* e) override;  
  void    drawText(QPainter &p, int x, int y, const QString &text, QColor qColor = QColor(255,255,255,255), int nAlign = Qt::AlignCenter );
  QColor  get_color( int nVal, int nRed, int nYellow );
  
private:
  UIState  *state;
  UIScene  *scene;



  struct _PARAM_
  {
    int   bbh_left;
    int   bbh_right;
    float altitudeUblox;
    float gpsAccuracyUblox;
    float bearingUblox;
    float  batteryTemp;
    float  angleSteers;
    float  angleSteersDes;
    
    int   cpuPerc;
    float cpuTemp; 
    
    cereal::RadarState::LeadData::Reader lead_radar;
    cereal::CarState::Reader car_state;    

  } m_param, m_old;
  

private:
  const int img_size = 200;// (radius / 2) * 1.5;
  const int img_size_compass = 300;

  

  QPixmap img_traf_turn;
  QPixmap img_compass;
  QPixmap img_direction;
  QPixmap img_tire_pressure;

  // speed
  QPixmap img_camera;
  QPixmap img_speed;
  QPixmap img_section;


  QPixmap img_speed_var;
  QPixmap img_img_space;
  QPixmap img_car_left;
  QPixmap img_car_right;
  QPixmap img_speed_bump;
  QPixmap img_bus_only;
  QPixmap img_school_zone;

  QPixmap img_curve_right;
  QPixmap img_curve_left;
  QPixmap img_narrow_road;
  QPixmap img_rail_road;

  QPixmap img_overtrack;
  QPixmap img_park_crackdown;


  // osm
  QPixmap map_img;
  QPixmap left_img;
  QPixmap right_img;

  const int radius = 192;
  bool showVTC = false;
  QString vtcSpeed;
  QColor vtcColor;
  bool showDebugUI = false;
  
  QString roadName;

  bool showSpeedLimit = false;
  QString speedLimit;
  QString slcSubText;
  float slcSubTextSize = 0.0;
  bool mapSourcedSpeedLimit = false;
  bool slcActive = false;

  bool showTurnSpeedLimit = false;
  QString turnSpeedLimit;
  QString tscSubText;
  bool tscActive = false;
  int curveSign = 0;

private:
  void drawCenteredText(QPainter &p, int x, int y, const QString &text, QColor color);
  void drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity);
  void drawVisionTurnControllerUI(QPainter &p, int x, int y, int size, const QColor &color, const QString &speed, 
                                  int alpha);
  void drawCircle(QPainter &p, int x, int y, int r, QBrush bg);
  void drawSpeedSign(QPainter &p, QRect rc, const QString &speed, const QString &sub_text, int subtext_size, 
                     bool is_map_sourced, bool is_active);
  void drawTrunSpeedSign(QPainter &p, QRect rc, const QString &speed, const QString &sub_text, int curv_sign, 
                         bool is_active);

  void ui_osm_draw( QPainter &p );

// navi
private:
  float  interp( float xv, float xp[], float fp[], int N);
  void   ui_main_navi( QPainter &p );
  void   ui_draw_debug1( QPainter &p );
  void   ui_draw_navi( QPainter &p );
  void   ui_draw_traffic_sign( QPainter &p, float map_sign, float dSpeed,  float speedLimitAheadDistance );
  int    get_param( const std::string &key );
// kegmen
private:

  //void ui_draw_circle_image_rotation(const UIState *s, int center_x, int center_y, int radius, const char *image, NVGcolor color, float img_alpha, float angleSteers = 0);
  int  bb_ui_draw_measure(QPainter &p,  const QString &bb_value, const QString &bb_uom, const QString &bb_label,
    int bb_x, int bb_y, int bb_uom_dx,
    QColor bb_valueColor, QColor bb_labelColor, QColor bb_uomColor,
    int bb_valueFontSize, int bb_labelFontSize, int bb_uomFontSize );


  void bb_ui_draw_measures_right(QPainter &p, int bb_x, int bb_y, int bb_w );
  void bb_ui_draw_measures_left(QPainter &p, int bb_x, int bb_y, int bb_w );

  QColor angleSteersColor( int angleSteers );
  QColor get_tpms_color(float tpms);
  QString get_tpms_text(float tpms);

  void  bb_draw_tpms(QPainter &p, int viz_tpms_x, int viz_tpms_y );
  void  bb_draw_compass(QPainter &p, int compass_x, int compass_y );

  void  bb_ui_draw_UI(QPainter &p);

signals:
  void valueChanged();  
};
