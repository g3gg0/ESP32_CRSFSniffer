
#include "SSD1306Wire.h" 
#include "Plot.h" 

PlotClass::PlotClass(SSD1306Wire *disp, const char *name, uint32_t groupSize, uint32_t groups) 
{
  Display = disp;
  strncpy(Name, name, 32);
  GroupSize = groupSize;
  Groups = groups;
  ValueCount = GroupSize * Groups;
  Values = new float[ValueCount];
}

void PlotClass::AddSample(float value)
{
  if(Entries < ValueCount)
  {
    Values[StartPos + Entries] = value;
    Entries++;
  }
  else
  {
    Values[StartPos] = value;
    StartPos = (StartPos + 1) % (ValueCount);
  }
}

void PlotClass::Clear()
{
  StartPos = 0;
  Entries = 0;
}

void PlotClass::CalcStats(float *minValue, float *maxValue, bool scatter)
{
  *minValue = 999999;
  *maxValue = -999999;
  
  if(scatter)
  {
    for(uint32_t pos = 0; pos < Entries; pos++)
    {
      float value = Values[(StartPos + pos) % (ValueCount)];
  
      if(value < *minValue)
      {
        *minValue = value;
      }
      if(value > *maxValue)
      {
        *maxValue = value;
      }
    }
  }
  else
  {
    for(uint32_t pos = 0; pos < Entries; pos += GroupSize)
    {
      float avgValue = 0;
      int entries = 0;
      for(uint32_t grp = 0; grp < GroupSize; grp++)
      {
        if(pos + grp < Entries)
        {
          avgValue += Values[(StartPos + pos + grp) % (ValueCount)];
          entries++;
        }
      }
      avgValue /= entries;
      if(avgValue < *minValue)
      {
        *minValue = avgValue;
      }
      if(avgValue > *maxValue)
      {
        *maxValue = avgValue;
      }
    }
  }

  float delta = *maxValue - *minValue;
  float extra = delta/5; 

  *maxValue += extra;
  *minValue -= extra;
}

void PlotClass::DrawGrid(int x, int y, int w, int h, int *xStart, int *yStart, int *xSpan, int *ySpan)
{
  Display->drawLine(x+2, y, x+2, y+h-1-2);
  Display->drawLine(x+2, y, x, y+2);
  Display->drawLine(x+2, y+h-1-2, x+w-1, y+h-1-2);
  Display->drawLine(x+w-1, y+h-1-2, x+w-1-2, y+h-1);

  *xStart = x+4;
  *yStart = y;
  *xSpan = w-4;
  *ySpan = h-4;
}

void PlotClass::DrawFullPlot(int x, int y, int w, int h, bool scatter)
{
  float maxValue = 0;
  float minValue = 0;

  CalcStats(&minValue, &maxValue, scatter);
  
  int xStart = 0;
  int yStart = 0;
  int xSpan = 0;
  int ySpan = 0;
  DrawGrid(x, y, w, h, &xStart, &yStart, &xSpan, &ySpan);

  if(scatter)
  {
    for(uint32_t pos = 0; pos < Entries; pos++)
    {
      float value = Values[(StartPos + pos) % (ValueCount)];
      float relValue = (value - minValue) / (maxValue - minValue);
      
      uint32_t xPos = xStart + ((float)pos / GroupSize) * xSpan / (float)Groups;
      uint32_t yPos = yStart + ySpan - relValue * ySpan;
      
      Display->setPixel(xPos, yPos);
    }
  }
  else
  {
    for(uint32_t pos = 0; pos < Entries; pos += GroupSize)
    {
      float avgValue = 0;
      int entries = 0;
      for(uint32_t grp = 0; grp < GroupSize; grp++)
      {
        if(pos + grp < Entries)
        {
          avgValue += Values[(StartPos + pos + grp) % (ValueCount)];
          entries++;
        }
      }
      avgValue /= entries;
      float relValue = (avgValue - minValue) / (maxValue - minValue);
      
      uint32_t xPos = xStart + ((float)pos / GroupSize) * xSpan / (float)Groups;
      uint32_t yPos = yStart + ySpan - relValue * ySpan;
      
      Display->setPixel(xPos, yPos);
    }
  }
  char msg[33];
  Display->setFont(ArialMT_Plain_10);
  snprintf(msg, 32, "%2.2f", maxValue);

  /* draw a black halo around the bright text */
  Display->setColor(BLACK);
  for(int mod = 0; mod <9; mod++)
  {
    Display->drawString(x+8 -1 + mod/3, y - 1 + mod %3, msg);
  }
  Display->setColor(WHITE);
  Display->drawString(x+8, y, msg);
  
  snprintf(msg, 32, "%2.2f", minValue);
  
  /* draw a black halo around the bright text */
  Display->setColor(BLACK);
  for(int mod = 0; mod <9; mod++)
  {
    Display->drawString(x+w+4 -1 + mod/3, y+4 - 1 + mod %3, msg);
  }
  Display->setColor(WHITE);
  Display->drawString(x+8, y+ySpan-10, msg);

  
  Display->setFont(ArialMT_Plain_16);
  Display->drawString(x+w+4, y+4, Name);
  
  Display->setFont(ArialMT_Plain_10);
  snprintf(msg, 32, "(# %d)", Entries);
  Display->drawString(x+w+4, y+16+4, msg);
}


