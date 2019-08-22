// CommonMethods.cpp: implementation of the CCommonMethods class.类型转换函数
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "CommonMethods.h"
#include <math.h>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CCommonMethods::CCommonMethods()
{

}

CCommonMethods::~CCommonMethods()
{

}

CString CCommonMethods::DoubleToStr(double dValue, int precision)
{
	if (fabs(dValue) < 0.0000000000000000001)
	{
		return _T("");
	}

	CString str;
	CString format;
	format.Format("%d",precision);
	format =  "%0." + format +"f";
	str.Format(format,dValue);
	
	return str;
}

CString CCommonMethods::DoubleToStr(double dValue)
{
	if (fabs(dValue) < 0.0000000000000000001)
	{
		return _T("");
	}
	
	CString str;
	str.Format("%lf",dValue);
	FormatFloatString(str);
	return str;
}

void CCommonMethods::FormatFloatString(CString &strFloat)
{
    if(strFloat.Find('.')<0)
        return;
	
    int        nIndex;
    char    aTemp;
	
    while(true)
    {
        nIndex    = strFloat.GetLength()-1;
        aTemp    = strFloat.GetAt(nIndex);
        if(aTemp=='.')
        {
            strFloat.Delete(nIndex);
            break;
        }
        if(aTemp!='0')
            break;
        strFloat.Delete(nIndex);
    }
}

double CCommonMethods::StrToDouble(CString sValue)
{
	double dValue;
	dValue = (double)atof(sValue);
	return dValue;
}

CString CCommonMethods::TimeToStr(COleDateTime time)
{
	return time.Format("%Y-%m-%d %H:%M:%S");
}

COleDateTime CCommonMethods::StrToTime(CString Str)
{
		COleDateTime tm;
		tm.ParseDateTime(Str);
		return tm;
}

CTime CCommonMethods::COleDateTimeToCTime(COleDateTime time)
{
	SYSTEMTIME systime;
	VariantTimeToSystemTime(time, &systime);
	CTime tm(systime);
	return tm;
}

COleDateTime CCommonMethods::CTimeToCOleDateTime(CTime time)
{
	time_t time2=time.GetTime(); //CTime—>time_t
	COleDateTime tm(time2); //time_t—>COleDateTime
	return tm;
}

int CCommonMethods::GetStrPosInArray(CStringArray* pArray, const CString str)
{   
	int nReturn = -1;   
	if(!pArray)   
		return nReturn;   
	ASSERT_VALID(pArray);   
	int i;
	int nCount = pArray->GetSize();   
	for(i=0; i<nCount; i++)   
	{   
		if((CString)pArray->GetAt(i) == str)   
		{   
			nReturn = i;   
			break;   
		}   
	}      
	return nReturn;   
}   

CString CCommonMethods::IntToStr(int iValue)
{
	CString str;
	str.Format("%d",iValue);
	return str;
}

int CCommonMethods::StrToInt(CString sValue)
{
	return atoi(sValue);
}

CString CCommonMethods::IntToHexStr(int nVal, BOOL bUpper)
{
	CString str = _T("");
	str.Format("%x",nVal);
	if (bUpper)
		str.MakeUpper();
	return str;
}