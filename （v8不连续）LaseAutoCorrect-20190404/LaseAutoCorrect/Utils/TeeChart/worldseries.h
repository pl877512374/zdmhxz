#if !defined(AFX_WORLDSERIES_H__F2D41A00_C0F8_41D2_8A5E_6AFD70E52033__INCLUDED_)
#define AFX_WORLDSERIES_H__F2D41A00_C0F8_41D2_8A5E_6AFD70E52033__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Machine generated IDispatch wrapper class(es) created by Microsoft Visual C++

// NOTE: Do not modify the contents of this file.  If this class is regenerated by
//  Microsoft Visual C++, your modifications will be overwritten.


// Dispatch interfaces referenced by this interface
class CValueList;
class CBrush1;
class CPen1;
class CMapPolygonList;
class CTeeShadow;

/////////////////////////////////////////////////////////////////////////////
// CWorldSeries wrapper class

class CWorldSeries : public COleDispatchDriver
{
public:
	CWorldSeries() {}		// Calls COleDispatchDriver default constructor
	CWorldSeries(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	CWorldSeries(const CWorldSeries& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	long AddXYZ(double AX, double AY, double AZ, LPCTSTR AXLabel, unsigned long Value);
	double MaxZValue();
	double MinZValue();
	long GetTimesZOrder();
	void SetTimesZOrder(long nNewValue);
	CValueList GetZValues();
	double GetZValue(long Index);
	void SetZValue(long Index, double newValue);
	CBrush1 GetBrush();
	CPen1 GetPen();
	void AddArrayXYZ(const VARIANT& XArray, const VARIANT& YArray, const VARIANT& ZArray);
	void AddArrayGrid(long NumX, long NumZ, const VARIANT& XZArray);
	long CalcZPos(long ValueIndex);
	long AddPalette(double Value, unsigned long Color);
	unsigned long GetStartColor();
	void SetStartColor(unsigned long newValue);
	unsigned long GetEndColor();
	void SetEndColor(unsigned long newValue);
	long GetPaletteSteps();
	void SetPaletteSteps(long nNewValue);
	BOOL GetUsePalette();
	void SetUsePalette(BOOL bNewValue);
	BOOL GetUseColorRange();
	void SetUseColorRange(BOOL bNewValue);
	void ClearPalette();
	void CreateDefaultPalette(long NumSteps);
	unsigned long GetSurfacePaletteColor(double Y);
	unsigned long GetMidColor();
	void SetMidColor(unsigned long newValue);
	void CreateRangePalette();
	long GetPaletteStyle();
	void SetPaletteStyle(long nNewValue);
	BOOL GetUsePaletteMin();
	void SetUsePaletteMin(BOOL bNewValue);
	double GetPaletteMin();
	void SetPaletteMin(double newValue);
	double GetPaletteStep();
	void SetPaletteStep(double newValue);
	void InvertPalette();
	void AddCustomPalette(const VARIANT& colorArray);
	CMapPolygonList GetShapes();
	long GetPointSize();
	void SetPointSize(long nNewValue);
	CTeeShadow GetShadow();
	BOOL GetTransparent();
	void SetTransparent(BOOL bNewValue);
	long GetTransparency();
	void SetTransparency(long nNewValue);
	long GetMap();
	void SetMap(long nNewValue);
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_WORLDSERIES_H__F2D41A00_C0F8_41D2_8A5E_6AFD70E52033__INCLUDED_)
