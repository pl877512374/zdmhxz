#if !defined(AFX_PANEL_H__23206447_C831_4217_B246_2AC9542DDD27__INCLUDED_)
#define AFX_PANEL_H__23206447_C831_4217_B246_2AC9542DDD27__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Machine generated IDispatch wrapper class(es) created by Microsoft Visual C++

// NOTE: Do not modify the contents of this file.  If this class is regenerated by
//  Microsoft Visual C++, your modifications will be overwritten.


// Dispatch interfaces referenced by this interface
class CGradient;
class CTeeShadow;
class CPen1;
class CBackImage;

/////////////////////////////////////////////////////////////////////////////
// CPanel wrapper class

class CPanel : public COleDispatchDriver
{
public:
	CPanel() {}		// Calls COleDispatchDriver default constructor
	CPanel(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	CPanel(const CPanel& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	BOOL GetBackImageInside();
	void SetBackImageInside(BOOL bNewValue);
	long GetBackImageMode();
	void SetBackImageMode(long nNewValue);
	long GetBevelInner();
	void SetBevelInner(long nNewValue);
	long GetBevelOuter();
	void SetBevelOuter(long nNewValue);
	long GetBevelWidth();
	void SetBevelWidth(long nNewValue);
	long GetBevelOffset();
	void SetBevelOffset(long nNewValue);
	long GetBorderStyle();
	void SetBorderStyle(long nNewValue);
	unsigned long GetColor();
	void SetColor(unsigned long newValue);
	CGradient GetGradient();
	long GetMarginLeft();
	void SetMarginLeft(long nNewValue);
	long GetMarginTop();
	void SetMarginTop(long nNewValue);
	long GetMarginRight();
	void SetMarginRight(long nNewValue);
	long GetMarginBottom();
	void SetMarginBottom(long nNewValue);
	void BackImageClear();
	void BackImageLoad(LPCTSTR FileName);
	long GetMarginUnits();
	void SetMarginUnits(long nNewValue);
	CTeeShadow GetShadow();
	long GetBorderRound();
	void SetBorderRound(long nNewValue);
	CPen1 GetBorder();
	BOOL GetBackImageTransparent();
	void SetBackImageTransparent(BOOL bNewValue);
	CBackImage GetBackImage();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_PANEL_H__23206447_C831_4217_B246_2AC9542DDD27__INCLUDED_)
