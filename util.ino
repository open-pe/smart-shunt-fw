void printTime() {
  char buffer[26];
  int millisec;
  struct timeval tv;

  gettimeofday(&tv, NULL);

  millisec = lrint(tv.tv_usec / 1000.0);  // Round to nearest millisec
  if (millisec >= 1000) {                 // Allow for rounding up to nearest second
    millisec -= 1000;
    tv.tv_sec++;
  }

  strftime(buffer, 26, "%H:%M:%S", localtime(&tv.tv_sec));
  // printf("%s.%03d\n", buffer, millisec);
  Serial.print(buffer);
  Serial.print('.');
  Serial.print(millisec);
  Serial.print(' ');

  /*
  char buff[100];
    time_t now = time (0);
    strftime (buff, 100, "%H:%M:%S.000", localtime (&now));
    printf ("%s\n", buff);
    return 0;
  */
}


void pointFromSample(Point &p, const Sample &s, const char *device) {
  point.addTag("device", device);
  point.addField("I", s.i, 3);
  point.addField("U", s.u, 3);
  point.addField("P", s.p(), 3);
  point.addField("E", s.e, 3);
  point.setTime(s.t);
}


class PointDefaultConstructor : public Point {
public:
  PointDefaultConstructor()
    : Point("smart_shut") {}
  PointDefaultConstructor(const Point &p)
    : Point(p) {}

  PointDefaultConstructor &operator=(const PointDefaultConstructor &p) {
    Point::operator=(p);
    return *this;
  }
};