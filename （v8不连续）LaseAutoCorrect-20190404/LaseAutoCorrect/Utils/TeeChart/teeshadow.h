#if !defined(AFX_TEESHADOW_H__33548E47_8C6B_4F79_87E2_21326685AAB1__INCLUDED_)
#define AFX_TEESHADOW_H__33548E47_8C6B_4F79_87E2_21326685AAB1__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Machine generated IDispatch wrapper class(es) created by Microsoft Visual C++

// NOTE: Do not modify the contents of this file.  If this class is regenerated by
//  Microsoft Visual C++, your modifications will be overwritten.

/////////////////////////////////////////////////////////////////////////////
// CTeeShadow wrapper class

class CTeeShadow : public COleDispatchDriver
{
public:
	CTeeShadow() {}		// Calls COleDispatchDriver default constructor
	CTeeShadow(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	CTeeShadow(const CTeeShadow& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	long GetHorizSize();
	void SetHorizSize(long nNewValue);
	long GetVertSize();
	void SetVertSize(long nNewValue);
	unsigned long GetColor();
	void SetColor(unsigned long newValue);
	long GetTransparency();
	void SetTransparency(long nNewValue);
	void ShowEditor();
	BOOL GetVisible();
	void SetVisible(BOOL bNewValue);
	BOOL GetSmooth();
	void SetSmooth(BOOL bNewValue);
	long GetSmoothBlur();
	void SetSmoothBlur(long nNewValue);
	BOOL GetClip();
	void SetClip(BOOL bNewValue);
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_TEESHADOW_H__33548E47_8C6B_4F79_87E2_21326685AAB1__INCLUDED_)
