#if !defined(AFX_RENKOSERIES_H__D2A98D0F_9916_4779_9CA3_E22AFFF5BBE1__INCLUDED_)
#define AFX_RENKOSERIES_H__D2A98D0F_9916_4779_9CA3_E22AFFF5BBE1__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Machine generated IDispatch wrapper class(es) created by Microsoft Visual C++

// NOTE: Do not modify the contents of this file.  If this class is regenerated by
//  Microsoft Visual C++, your modifications will be overwritten.


// Dispatch interfaces referenced by this interface
class CValueList;

/////////////////////////////////////////////////////////////////////////////
// CRenkoSeries wrapper class

class CRenkoSeries : public COleDispatchDriver
{
public:
	CRenkoSeries() {}		// Calls COleDispatchDriver default constructor
	CRenkoSeries(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	CRenkoSeries(const CRenkoSeries& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	CValueList GetCloseValues();
	double GetBoxSize();
	void SetBoxSize(double newValue);
	unsigned long GetDownSwingColor();
	void SetDownSwingColor(unsigned long newValue);
	BOOL GetDraw3D();
	void SetDraw3D(BOOL bNewValue);
	long GetTransparency();
	void SetTransparency(long nNewValue);
	unsigned long GetUpSwingColor();
	void SetUpSwingColor(unsigned long newValue);
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_RENKOSERIES_H__D2A98D0F_9916_4779_9CA3_E22AFFF5BBE1__INCLUDED_)
