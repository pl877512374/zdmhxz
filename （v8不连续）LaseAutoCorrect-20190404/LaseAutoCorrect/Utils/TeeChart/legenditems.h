#if !defined(AFX_LEGENDITEMS_H__B7AE2365_7DB5_4473_B20E_E45FD63D7D04__INCLUDED_)
#define AFX_LEGENDITEMS_H__B7AE2365_7DB5_4473_B20E_E45FD63D7D04__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Machine generated IDispatch wrapper class(es) created by Microsoft Visual C++

// NOTE: Do not modify the contents of this file.  If this class is regenerated by
//  Microsoft Visual C++, your modifications will be overwritten.


// Dispatch interfaces referenced by this interface
class CLegendItem;

/////////////////////////////////////////////////////////////////////////////
// CLegendItems wrapper class

class CLegendItems : public COleDispatchDriver
{
public:
	CLegendItems() {}		// Calls COleDispatchDriver default constructor
	CLegendItems(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	CLegendItems(const CLegendItems& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	void Clear();
	BOOL GetCustom();
	void SetCustom(BOOL bNewValue);
	CLegendItem GetItems(long Index);
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_LEGENDITEMS_H__B7AE2365_7DB5_4473_B20E_E45FD63D7D04__INCLUDED_)
