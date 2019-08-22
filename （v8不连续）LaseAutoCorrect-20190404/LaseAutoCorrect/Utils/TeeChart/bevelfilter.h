#if !defined(AFX_BEVELFILTER_H__5E9A6FF0_2DE5_4C4F_8E43_DF595881A149__INCLUDED_)
#define AFX_BEVELFILTER_H__5E9A6FF0_2DE5_4C4F_8E43_DF595881A149__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Machine generated IDispatch wrapper class(es) created by Microsoft Visual C++

// NOTE: Do not modify the contents of this file.  If this class is regenerated by
//  Microsoft Visual C++, your modifications will be overwritten.

/////////////////////////////////////////////////////////////////////////////
// CBevelFilter wrapper class

class CBevelFilter : public COleDispatchDriver
{
public:
	CBevelFilter() {}		// Calls COleDispatchDriver default constructor
	CBevelFilter(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	CBevelFilter(const CBevelFilter& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	long GetBright();
	void SetBright(long nNewValue);
	long GetSize();
	void SetSize(long nNewValue);
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_BEVELFILTER_H__5E9A6FF0_2DE5_4C4F_8E43_DF595881A149__INCLUDED_)
