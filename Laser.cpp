// See LICENSE file for details
// Copyright 2016 Florian Link (at) gmx.de
#include "Laser.h"


Laser::Laser(uint8_t galvoXPin, uint8_t galvoYPin, uint8_t laserPin) {
  _laserPin = laserPin;
  _quality = FROM_FLOAT(1. / (LASER_QUALITY));

  _x = 0;
  _y = 0;
  _oldX = 0;
  _oldY = 0;

  _state = 0;

  _scale = 1;
  _offsetX = 0;
  _offsetY = 0;

  _moved = 0;
  _maxMove = -1;
  _laserForceOff = false;
  resetClipArea();

  _enable3D = false;
  _zDist = 1000;
  _is_rgb = false;
}

Laser::Laser(uint8_t galvoXPin, uint8_t galvoYPin, uint8_t redPin, uint8_t greenPin, uint8_t bluePin, bool is_pwm)
  : _rgb_pins{ redPin, greenPin, bluePin },
    _galvo_pin_x(galvoXPin),
    _galvo_pin_y(galvoYPin),
    _is_pwm(is_pwm) {
  _quality = FROM_FLOAT(1. / (LASER_QUALITY));

  _x = 0;
  _y = 0;
  _oldX = 0;
  _oldY = 0;

  _state = 0;

  _scale = 1;
  _offsetX = 0;
  _offsetY = 0;

  _moved = 0;
  _maxMove = -1;
  _laserForceOff = false;
  resetClipArea();

  _enable3D = false;
  _zDist = 1000;
  _is_rgb = true;
  _current_color = RGBColor{ 1, 1, 1 };
}

void Laser::init() {
  _galvo_pwm_instance_x = new RP2040_PWM(_galvo_pin_x, GALVO_FREQ, 0.0f);  // Start with 0% duty cycle
  _galvo_pwm_instance_y = new RP2040_PWM(_galvo_pin_y, GALVO_FREQ, 0.0f);  // Start with 0% duty cycle

  _galvo_top = _galvo_pwm_instance_x->get_TOP();
  _galvo_div = _galvo_pwm_instance_x->get_DIV();
  _PWM_Level = 0;

  _galvo_pwm_instance_x->setPWM_manual(_galvo_pin_x, _galvo_top, _galvo_div, _PWM_Level);
  _galvo_pwm_instance_y->setPWM_manual(_galvo_pin_y, _galvo_top, _galvo_div, _PWM_Level);

  if (_is_rgb) {
    if (_is_pwm)
    {
      for (int i = 0; i < 3; i++) {
        _rgb_pwm_channels[i] = new RP2040_PWM(_rgb_pins[i], RGB_FREQ, 0.0f);
      }

      _rgb_top = _rgb_pwm_channels[0]->get_TOP();
      _rgb_div = _rgb_pwm_channels[0]->get_DIV();

      // Initialize PWM channels
      for (int i = 0; i < 3; i++) {
        _rgb_pwm_channels[i]->setPWM_manual(_rgb_pins[i], _rgb_top, _rgb_div, _PWM_Level);
      }
    }
    else
    {
      for (int i = 0; i < 3; i++) {
        pinMode(_rgb_pins[i], OUTPUT);
      }
    }
  }
}

void Laser::setColor(RGBColor color) {
  if (color == _current_color) {
    return;
  }
  if (_state) {
    if (_is_pwm)
    {
      uint16_t dutyCycles[3];
      dutyCycles[0] = map(color.red, 0, 255, 0, _rgb_top);
      dutyCycles[1] = map(color.green, 0, 255, 0, _rgb_top);
      dutyCycles[2] = map(color.blue, 0, 255, 0, _rgb_top);

      for (int i = 0; i < 3; i++) {
        _rgb_pwm_channels[i]->setPWM_manual_Fast(_rgb_pins[i], dutyCycles[i]);
      }
    }
    else
    {
      for (int i = 0; i < 3; i++) {
        digitalWrite(_rgb_pins[i], color.colors[i]);
      }
    }
  }
  _current_color = color;
}

void Laser::SendToDAC(int x, int y) {
#ifdef LASER_SWAP_XY
  int x1 = y;
  int y1 = x;
#else
  int x1 = x;
  int y1 = y;
#endif
#ifdef LASER_FLIP_X
  x1 = 4095 - x1;
#endif
#ifdef LASER_FLIP_Y
  y1 = 4095 - y1;
#endif

  // Directly compute the duty cycles using clamped values
  uint16_t dutyCycleX = map(x1, 0, 4095, 0, _galvo_top);
  uint16_t dutyCycleY = map(y1, 0, 4095, 0, _galvo_top);


  _galvo_pwm_instance_x->setPWM_manual_Fast(_galvo_pin_x, dutyCycleX);
  _galvo_pwm_instance_y->setPWM_manual_Fast(_galvo_pin_y, dutyCycleY);
}

void Laser::resetClipArea() {
  _clipXMin = 0;
  _clipYMin = 0;
  _clipXMax = 4095;
  _clipYMax = 4095;
}

void Laser::setClipArea(long x, long y, long x1, long y1) {
  _clipXMin = x;
  _clipYMin = y;
  _clipXMax = x1;
  _clipYMax = y1;
}

const int INSIDE = 0;  // 0000
const int LEFT = 1;    // 0001
const int RIGHT = 2;   // 0010
const int BOTTOM = 4;  // 0100
const int TOP = 8;     // 1000

int Laser::computeOutCode(long x, long y) {
  int code = INSIDE;  // initialised as being inside of [[clip window]]

  if (x < _clipXMin)  // to the left of clip window
    code |= LEFT;
  else if (x > _clipXMax)  // to the right of clip window
    code |= RIGHT;
  if (y < _clipYMin)  // below the clip window
    code |= BOTTOM;
  else if (y > _clipYMax)  // above the clip window
    code |= TOP;

  return code;
}

// Cohen–Sutherland clipping algorithm clips a line from
// P0 = (x0, y0) to P1 = (x1, y1) against a rectangle with
// diagonal from (_clipXMin, _clipYMin) to (_clipXMax, _clipYMax).
bool Laser::clipLine(long& x0, long& y0, long& x1, long& y1) {
  // compute outcodes for P0, P1, and whatever point lies outside the clip rectangle
  int outcode0 = computeOutCode(x0, y0);
  int outcode1 = computeOutCode(x1, y1);
  bool accept = false;

  while (true) {
    if (!(outcode0 | outcode1)) {  // Bitwise OR is 0. Trivially accept and get out of loop
      accept = true;
      break;
    } else if (outcode0 & outcode1) {  // Bitwise AND is not 0. Trivially reject and get out of loop
      break;
    } else {
      // failed both tests, so calculate the line segment to clip
      // from an outside point to an intersection with clip edge
      long x, y;

      // At least one endpoint is outside the clip rectangle; pick it.
      int outcodeOut = outcode0 ? outcode0 : outcode1;

      // Now find the intersection point;
      // use formulas y = y0 + slope * (x - x0), x = x0 + (1 / slope) * (y - y0)
      if (outcodeOut & TOP) {  // point is above the clip rectangle
        x = x0 + (x1 - x0) * float(_clipYMax - y0) / float(y1 - y0);
        y = _clipYMax;
      } else if (outcodeOut & BOTTOM) {  // point is below the clip rectangle
        x = x0 + (x1 - x0) * float(_clipYMin - y0) / float(y1 - y0);
        y = _clipYMin;
      } else if (outcodeOut & RIGHT) {  // point is to the right of clip rectangle
        y = y0 + (y1 - y0) * float(_clipXMax - x0) / float(x1 - x0);
        x = _clipXMax;
      } else if (outcodeOut & LEFT) {  // point is to the left of clip rectangle
        y = y0 + (y1 - y0) * float(_clipXMin - x0) / float(x1 - x0);
        x = _clipXMin;
      }

      // Now we move outside point to intersection point to clip
      // and get ready for next pass.
      if (outcodeOut == outcode0) {
        x0 = x;
        y0 = y;
        outcode0 = computeOutCode(x0, y0);
      } else {
        x1 = x;
        y1 = y;
        outcode1 = computeOutCode(x1, y1);
      }
    }
  }
  return accept;
}

void Laser::SendTo(long xpos, long ypos) {
  if (_enable3D) {
    Vector3i p1;
    Vector3i p;
    p1.x = xpos;
    p1.y = ypos;
    p1.z = 0;
    Matrix3::applyMatrix(_matrix, p1, p);
    xpos = ((_zDist * (long)p.x) / (_zDist + (long)p.z)) + 2048;
    ypos = ((_zDist * (long)p.y) / (_zDist + (long)p.z)) + 2048;
  }
  // Float was too slow on Arduino, so I used
  // fixed point precision here:
  long xNew = TO_INT(xpos * _scale) + _offsetX;
  long yNew = TO_INT(ypos * _scale) + _offsetY;
  long clipX = xNew;
  long clipY = yNew;
  long oldX = _oldX;
  long oldY = _oldY;
  if (clipLine(oldX, oldY, clipX, clipY)) {
    if (oldX != _oldX || oldY != _oldY) {
      SendToRaw(oldX, oldY);
    }
    SendToRaw(clipX, clipY);
  }
  _oldX = xNew;
  _oldY = yNew;
}

void Laser::SendToRaw(long xNew, long yNew) {
  // devide into equal parts, using _quality
  long fdiffx = xNew - _x;
  long fdiffy = yNew - _y;
  long diffx = TO_INT(abs(fdiffx) * _quality);
  long diffy = TO_INT(abs(fdiffy) * _quality);

  // store movement for max move
  long moved = _moved;
  _moved += abs(fdiffx) + abs(fdiffy);

  // use the bigger direction
  if (diffx < diffy) {
    diffx = diffy;
  }
  fdiffx = FROM_INT(fdiffx) / diffx;
  fdiffy = FROM_INT(fdiffy) / diffx;
  // interpolate in FIXPT
  FIXPT tmpx = 0;
  FIXPT tmpy = 0;
  for (int i = 0; i < diffx - 1; i++) {
    // for max move, stop inside of line if required...
    if (_maxMove != -1) {
      long moved2 = moved + abs(TO_INT(tmpx)) + abs(TO_INT(tmpy));
      if (!_laserForceOff && moved2 > _maxMove) {
        off();
        _laserForceOff = true;
        _maxMoveX = _x + TO_INT(tmpx);
        _maxMoveY = _y + TO_INT(tmpy);
      }
    }
    tmpx += fdiffx;
    tmpy += fdiffy;
    SendToDAC(_x + TO_INT(tmpx), _y + TO_INT(tmpy));
#ifdef LASER_MOVE_DELAY
    wait(LASER_MOVE_DELAY);
#endif
  }

  // for max move, stop if required...
  if (!_laserForceOff && _maxMove != -1 && _moved > _maxMove) {
    off();
    _laserForceOff = true;
    _maxMoveX = xNew;
    _maxMoveY = yNew;
  }

  _x = xNew;
  _y = yNew;
  SendToDAC(_x, _y);
  wait(LASER_END_DELAY);
}

void Laser::drawLine(long x1, long y1, long x2, long y2, RGBColor color) {
  if (_x != x1 or _y != y1) {
    off();
    SendTo(x1, y1);
  }
  on(color);
  SendTo(x2, y2);
  wait(LASER_LINE_END_DELAY);
}

void Laser::drawRect(long x1, long y1, long x2, long y2, RGBColor color) {
  if (_x != x1 or _y != y1) {
    off();
    SendTo(x1, y1);
  }
  on(color);
  SendTo(x2, y1);
  SendTo(x2, y2);
  SendTo(x1, y2);
  SendTo(x1, y1);
}

void Laser::on(RGBColor color) {
  if (!color.empty() && ! (color == _current_color)) {
    _current_color = color;
  }

  if (!_state && !_laserForceOff) {
    wait(LASER_TOGGLE_DELAY);
    _state = 1;
    if (_is_rgb) {
      if (_is_pwm)
      {
        uint16_t dutyCycles[3];
        dutyCycles[0] = map(_current_color.red, 0, 255, 0, _rgb_top);
        dutyCycles[1] = map(_current_color.green, 0, 255, 0, _rgb_top);
        dutyCycles[2] = map(_current_color.blue, 0, 255, 0, _rgb_top);

        for (int i = 0; i < 3; i++) {
          _rgb_pwm_channels[i]->setPWM_manual_Fast(_rgb_pins[i], dutyCycles[i]);
        }
      }
      else
      {
        for (int i = 0; i < 3; i++)
        {
          digitalWrite(_rgb_pins[i], _current_color.colors[i]);
        }
      }
    }
    else {
      digitalWrite(_laserPin, HIGH);
    }
  }
}

void Laser::off() {
  if (_state) {
    wait(LASER_TOGGLE_DELAY);
    _state = 0;
    if(_is_rgb)
    {
      if (_is_pwm)
      {
        for (int i = 0; i < 3; i++) {
          _rgb_pwm_channels[i]->setPWM_manual_Fast(_rgb_pins[i], _PWM_Level);
        }
      }
      else
      {
        for (int i = 0; i < 3; i++) {
          digitalWrite(_rgb_pins[i], 0);
        }
      }
    }
    else {
      digitalWrite(_laserPin, LOW);
    }
  }
}

void Laser::wait(long length) {
  delayMicroseconds(length);
}

void Laser::setScale(float scale) {
  _scale = FROM_FLOAT(scale);
}

void Laser::setOffset(long offsetX, long offsetY) {
  _offsetX = offsetX;
  _offsetY = offsetY;
}

RGBColor Laser::hsvToRgb(float h, float s, float v) {
    int i = static_cast<int>(h * 6);
    float f = h * 6 - i;
    uint8_t p = static_cast<uint8_t>(255 * v * (1 - s));
    uint8_t q = static_cast<uint8_t>(255 * v * (1 - f * s));
    uint8_t t = static_cast<uint8_t>(255 * v * (1 - (1 - f) * s));
    uint8_t v_int = static_cast<uint8_t>(v * 255);  // Renamed to v_int to avoid name shadowing
    switch (i % 6) {
        case 0: return RGBColor{v_int, t, p};
        case 1: return RGBColor{q, v_int, p};
        case 2: return RGBColor{p, v_int, t};
        case 3: return RGBColor{p, q, v_int};
        case 4: return RGBColor{t, p, v_int};
        case 5: return RGBColor{v_int, p, q};
        default: return RGBColor{0, 0, 0}; // should never happen
    }
}
