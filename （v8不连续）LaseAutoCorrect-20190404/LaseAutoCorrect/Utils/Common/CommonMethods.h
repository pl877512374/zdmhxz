// CommonMethods.h: interface for the CCommonMethods class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_COMMONMETHODS_H__4BFD5C1C_A849_46B0_92B4_F406A98AE621__INCLUDED_)
#define AFX_COMMONMETHODS_H__4BFD5C1C_A849_46B0_92B4_F406A98AE621__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CCommonMethods  
{
public:
	CCommonMethods();
	virtual ~CCommonMethods();

public:
	static CString DoubleToStr(double dValue, int precision);
	static CString DoubleToStr(double dValue);
	static double StrToDouble(CString sValue);
	static CString TimeToStr(COleDateTime time);
	static COleDateTime StrToTime(CString Str);
	static CTime COleDateTimeToCTime(COleDateTime time);
	static COleDateTime CTimeToCOleDateTime(CTime time);
	static int GetStrPosInArray(CStringArray* pArray, const CString str);
	static CString IntToStr(int iValue);
	static int StrToInt(CString sValue);

	static CString IntToHexStr(int nVal, BOOL bUpper=FALSE);

private:
	static void FormatFloatString(CString &strFloat);

};

#endif // !defined(AFX_COMMONMETHODS_H__4BFD5C1C_A849_46B0_92B4_F406A98AE621__INCLUDED_)
