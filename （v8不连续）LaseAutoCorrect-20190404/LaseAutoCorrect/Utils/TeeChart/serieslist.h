#if !defined(AFX_SERIESLIST_H__4D3ECF67_6581_415E_A15C_8025428CE346__INCLUDED_)
#define AFX_SERIESLIST_H__4D3ECF67_6581_415E_A15C_8025428CE346__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Machine generated IDispatch wrapper class(es) created by Microsoft Visual C++

// NOTE: Do not modify the contents of this file.  If this class is regenerated by
//  Microsoft Visual C++, your modifications will be overwritten.


// Dispatch interfaces referenced by this interface
class CSeriesGroups;
class CSeries;

/////////////////////////////////////////////////////////////////////////////
// CSeriesList wrapper class

class CSeriesList : public COleDispatchDriver
{
public:
	CSeriesList() {}		// Calls COleDispatchDriver default constructor
	CSeriesList(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	CSeriesList(const CSeriesList& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	void AddGroup(LPCTSTR Name);
	BOOL GetAllActive();
	void SetAllActive(BOOL bNewValue);
	CSeriesGroups GetGroups();
	void ClearValues();
	long First();
	long Last();
	CSeries GetItems(long Index);
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_SERIESLIST_H__4D3ECF67_6581_415E_A15C_8025428CE346__INCLUDED_)
