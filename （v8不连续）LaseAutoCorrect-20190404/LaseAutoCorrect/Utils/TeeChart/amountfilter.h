#if !defined(AFX_AMOUNTFILTER_H__11AD2070_FCF9_46F7_8220_CC68E9DE255A__INCLUDED_)
#define AFX_AMOUNTFILTER_H__11AD2070_FCF9_46F7_8220_CC68E9DE255A__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Machine generated IDispatch wrapper class(es) created by Microsoft Visual C++

// NOTE: Do not modify the contents of this file.  If this class is regenerated by
//  Microsoft Visual C++, your modifications will be overwritten.

/////////////////////////////////////////////////////////////////////////////
// CAmountFilter wrapper class

class CAmountFilter : public COleDispatchDriver
{
public:
	CAmountFilter() {}		// Calls COleDispatchDriver default constructor
	CAmountFilter(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	CAmountFilter(const CAmountFilter& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	BOOL GetPercent();
	void SetPercent(BOOL bNewValue);
	long GetAmount();
	void SetAmount(long nNewValue);
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_AMOUNTFILTER_H__11AD2070_FCF9_46F7_8220_CC68E9DE255A__INCLUDED_)
