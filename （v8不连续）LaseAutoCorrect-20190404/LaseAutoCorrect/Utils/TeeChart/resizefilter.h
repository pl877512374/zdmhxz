#if !defined(AFX_RESIZEFILTER_H__56E5933C_8067_494B_866B_9742F60A82F4__INCLUDED_)
#define AFX_RESIZEFILTER_H__56E5933C_8067_494B_866B_9742F60A82F4__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Machine generated IDispatch wrapper class(es) created by Microsoft Visual C++

// NOTE: Do not modify the contents of this file.  If this class is regenerated by
//  Microsoft Visual C++, your modifications will be overwritten.

/////////////////////////////////////////////////////////////////////////////
// CResizeFilter wrapper class

class CResizeFilter : public COleDispatchDriver
{
public:
	CResizeFilter() {}		// Calls COleDispatchDriver default constructor
	CResizeFilter(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	CResizeFilter(const CResizeFilter& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	long GetWidth();
	void SetWidth(long nNewValue);
	long GetHeight();
	void SetHeight(long nNewValue);
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_RESIZEFILTER_H__56E5933C_8067_494B_866B_9742F60A82F4__INCLUDED_)
