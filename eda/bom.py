#!/usr/bin/env python

import sys
import csv
import libxml2
import libxslt
import tempfile
import StringIO
import re

digits = re.compile(r'(\d+)')
def tokenize(filename):
    return tuple(int(token) if match else token
                 for token, match in
                 ((fragment, digits.search(fragment))
                  for fragment in digits.split(filename)))

def bom(net_fname, csv_fname):

    styledoc = libxml2.parseDoc("""<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform" version="1.0">
    <xsl:output method="text"/>

    <!-- for table head and empty table fields-->
    <xsl:key name="headentr" match="field" use="@name"/>

    <!-- main part -->
    <xsl:template match="/export">
        <xsl:text>Reference,Value,Footprint,Datasheet</xsl:text>

            <!-- find all existing table head entries and list each one once -->
            <xsl:for-each select="components/comp/fields/field[generate-id(.) = generate-id(key('headentr',@name)[1])]">
                <xsl:text>,</xsl:text>
                <xsl:value-of select="@name"/>
            </xsl:for-each>
            <xsl:text>&#xa;</xsl:text>

        <!-- all table entries -->
        <xsl:apply-templates select="components/comp"/>
    </xsl:template>

    <!-- the table entries -->
    <xsl:template match="components/comp">
        <xsl:value-of select="@ref"/><xsl:text>,</xsl:text>
        <xsl:value-of select="value"/><xsl:text>,</xsl:text>
        <xsl:value-of select="footprint"/><xsl:text>,</xsl:text>
        <xsl:value-of select="datasheet"/>
        <xsl:apply-templates select="fields"/>
        <xsl:text>&#xa;</xsl:text>
    </xsl:template>

    <!-- table entries with dynamic table head -->
    <xsl:template match="fields">

        <!-- remember current fields section -->
        <xsl:variable name="fieldvar" select="field"/>

        <!-- for all existing head entries -->
        <xsl:for-each select="/export/components/comp/fields/field[generate-id(.) = generate-id(key('headentr',@name)[1])]">
            <xsl:variable name="allnames" select="@name"/>
            <xsl:text>,</xsl:text>

            <!-- for all field entries in the remembered fields section -->
            <xsl:for-each select="$fieldvar">

                <!-- only if this field entry exists in this fields section -->
                <xsl:if test="@name=$allnames">
                    <!-- content of the field -->
                    <xsl:value-of select="."/>
                </xsl:if>
                <!--
                    If it does not exist, use an empty cell in output for this row.
                    Every non-blank entry is assigned to its proper column.
                -->
            </xsl:for-each>
        </xsl:for-each>
    </xsl:template>

</xsl:stylesheet>
""")
    style    = libxslt.parseStylesheetDoc(styledoc)
    doc      = libxml2.parseFile(net_fname)
    result   = style.applyStylesheet(doc, None)

    out = style.saveResultToString(result)
    style.freeStylesheet()
    doc.freeDoc()
    result.freeDoc()

    rd = csv.DictReader(StringIO.StringIO(out))

    if csv_fname == '-':
        ofile = sys.stdout
    else:
        ofile = open(csv_fname, 'wb')


    bom = {}

    for e in rd:

        ds = e['mouser']

        if not ds or ds.lower() == 'na':
            continue

        if ds in bom:
            ref = bom[ds][0] + [e['Reference'], ]
            ref.sort(key=tokenize)
            qte = bom[ds][1] + 1
        else:
            ref = [e['Reference'], ]
            qte = 1

        bom[ds] = [ref, qte, e['Value'], e['Footprint'], e['Datasheet']]


    keys=bom.keys()
    keys.sort(key=tokenize)

    wr = csv.DictWriter(ofile, ['Mouser Ref', 'Qte', 'Sch Ref', 'Value', 'Footprint', 'Datasheet'])
    wr.writeheader()
    for k in keys:
        v = bom[k]
        wr.writerow({'Mouser Ref' : k,
                     'Qte'        : v[1],
                     'Sch Ref'    : ' '.join(v[0]),
                     'Value'      : v[2],
                     'Footprint'  : v[3],
                     'Datasheet'  : v[4]})


if __name__ == '__main__':

    bom(sys.argv[1], sys.argv[2])
