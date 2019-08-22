#if !defined(AFX_PRINTER_H__4919AFDE_8273_4EC1_A82E_00CAB7484968__INCLUDED_)
#define AFX_PRINTER_H__4919AFDE_8273_4EC1_A82E_00CAB7484968__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Machine generated IDispatch wrapper class(es) created by Microsoft Visual C++

// NOTE: Do not modify the contents of this file.  If this class is regenerated by
//  Microsoft Visual C++, your modifications will be overwritten.


// Dispatch interfaces referenced by this interface
class CPrinterSettings;

/////////////////////////////////////////////////////////////////////////////
// CPrinter wrapper class

class CPrinter : public COleDispatchDriver
{
public:
	CPrinter() {}		// Calls COleDispatchDriver default constructor
	CPrinter(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	CPrinter(const CPrinter& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	long GetDetail();
	void SetDetail(long nNewValue);
	long GetMarginBottom();
	void SetMarginBottom(long nNewValue);
	long GetMarginLeft();
	void SetMarginLeft(long nNewValue);
	long GetMarginTop();
	void SetMarginTop(long nNewValue);
	long GetMarginRight();
	void SetMarginRight(long nNewValue);
	long GetOrientation();
	void SetOrientation(long nNewValue);
	void Print();
	void PrintLandscape();
	void PrintPortrait();
	void ShowPreview();
	void PrintPartial(long Left, long Top, long Right, long Bottom);
	void BeginDoc();
	void EndDoc();
	long GetPrinterIndex();
	void SetPrinterIndex(long nNewValue);
	long GetPrinterCount();
	CString GetPrinterDescription(long Index);
	void PrintChart();
	long GetPageHeight();
	long GetPageWidth();
	CString GetJobTitle();
	void SetJobTitle(LPCTSTR lpszNewValue);
	BOOL GetPrintProportional();
	void SetPrintProportional(BOOL bNewValue);
	void PrintPartialHandle(const VARIANT& DC, long Left, long Top, long Right, long Bottom);
	void PrintPages(long FromPage, long ToPage);
	void ShowSetupDlg();
	CPrinterSettings GetPrinterSettings();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_PRINTER_H__4919AFDE_8273_4EC1_A82E_00CAB7484968__INCLUDED_)
