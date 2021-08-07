

class PlotClass
{
  private:
  SSD1306Wire *Display;
  char Name[33];
  float *Values;
  uint32_t StartPos = 0;
  uint32_t Entries = 0;
  uint32_t GroupSize = 0;
  uint32_t Groups = 0;
  uint32_t ValueCount = 0;

  public:
  PlotClass(SSD1306Wire *disp, const char *name, uint32_t groupSize, uint32_t groups);
  void AddSample(float value);
  void Clear();
  void DrawFullPlot(int x, int y, int w, int h, bool scatter = true);

  private:
  void DrawGrid(int x, int y, int w, int h, int *xStart, int *yStart, int *xSpan, int *ySpan);
  void CalcStats(float *minValue, float *maxValue, bool scatter);
};

