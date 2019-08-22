#if !defined(AFX_METAFILEEXPORT_H__301F6C79_5614_4851_BB86_E1829CC327F9__INCLUDED_)
#define AFX_METAFILEEXPORT_H__301F6C79_5614_4851_BB86_E1829CC327F9__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Machine generated IDispatch wrapper class(es) created by Microsoft Visual C++

// NOTE: Do not modify the contents of this file.  If this class is regenerated by
//  Microsoft Visual C++, your modifications will be overwritten.

/////////////////////////////////////////////////////////////////////////////
// CMetafileExport wrapper class

class CMetafileExport : public COleDispatchDriver
{
public:
	CMetafileExport() {}		// Calls COleDispatchDriver default constructor
	CMetafileExport(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	CMetafileExport(const CMetafileExport& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	long GetWidth();
	void SetWidth(long nNewValue);
	long GetHeight();
	void SetHeight(long nNewValue);
	void SaveToFile(LPCTSTR FileName);
	VARIANT SaveToStream();
	BOOL GetEnhanced();
	void SetEnhanced(BOOL bNewValue);
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_METAFILEEXPORT_H__301F6C79_5614_4851_BB86_E1829CC327F9__INCLUDED_)
