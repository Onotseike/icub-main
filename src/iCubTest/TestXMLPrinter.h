// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Author Alessandro Scalzo alessandro@liralab.it
 */

#ifndef __ICUB_TEST_XML_PRINTER_01122009__
#define __ICUB_TEST_XML_PRINTER_01122009__

#include <vector>
#include <string>

class XMLPrinter
{
public:
    XMLPrinter(std::string& filename)
    {
        m_pFile=fopen(filename.c_str(),"wb");

        m_Tabs=0;
        m_Stack.clear();
    }
    
    ~XMLPrinter()
    {
        if (m_pFile)
        {
            fclose(m_pFile);
        }
        m_pFile=0;
        m_Stack.clear();
    }
    
    void xml(char* tag,std::string data)
    {
        tabs();
        fprintf(m_pFile,"<%s>%s</%s>\n",tag,data.c_str(),tag);
    }
    void xmlOpen(char *tag)
    {
        tabs(1);
        m_Stack.push_back(std::string(tag));
        fprintf(m_pFile,"<%s>\n",tag);
    }
    void xmlClose()
    {
        tabs(-1);
        fprintf(m_pFile,"</%s>\n",m_Stack.back().c_str());
        m_Stack.pop_back();
    }

protected:
    FILE *m_pFile;
    int m_Tabs;
    std::vector<std::string> m_Stack;

    void tabs(int move=0)
    {
        if (move<0)
        {
            m_Tabs+=move;
        }

        for (int t=0; t<m_Tabs; ++t)
        {
            fprintf(m_pFile,"\t");
        }

        if (move>0) 
        {
            m_Tabs+=move;
        }
    }
};

#endif