/*
 * Copyright (c) 2009 Andriy Stefanov <dejan.pangercic -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Andriy Stefanov

@b Visual XML is a tool, which visualizes XML files as binary trees

 **/

import java.io.File;
import java.util.Vector;
import org.w3c.dom.Document;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

/**
 * parses a xml file and creates a Tree object
 * @author andriy
 */
public class Parser {
    
    public static Tree buildup(Tree tree,Node node){


		NodeList nodelist = node.getChildNodes();

		Tree result = new Tree(node.getNodeName().split("\n"),tree.depth+1);

		for (int i = 0; i < nodelist.getLength(); i++){

			Node fstnode = nodelist.item(i);

			String val = (fstnode.getNodeValue() == null) ? "" : fstnode.getNodeValue();
			String n = val.replaceAll(" ", "").replaceAll("\n", "").replaceAll("\t", "");

			if(!fstnode.hasChildNodes() && !(n.isEmpty())){
				result.addChild(new Tree(cleanString(fstnode.getNodeValue().split("\n")),tree.depth));
			}

			if (fstnode.getNodeType() == Node.ELEMENT_NODE) {

				if(fstnode.hasChildNodes()){
					result.addChild(buildup(result, fstnode));
				} else {
					result.addChild(new Tree(fstnode.getNodeName().split("\n"),result.depth));
				}
			}

		}

		return result;
	}

    /**
     * cleans the string of tabs, spaces and new lines
     * @param s "dirty" string array
     * @return clean string array
     */
    public static String[] cleanString(String[] s){
        Vector v =  new Vector();
        for(int i = 0; i < s.length; i++){
            String n = s[i].replaceAll(" ", "").replaceAll("\n", "").replaceAll("\t", "");
            
            while(s[i].startsWith(" ")){
                s[i] = s[i].substring(1);
            }
            if(!n.isEmpty()){
                v.addElement(s[i].replaceAll("\t", "").replaceAll("\n", ""));
            }
        }
        String[] result = new String[v.size()];
        for(int i = 0; i < v.size(); i++){
            result[i] = (String)v.elementAt(i);
        }
        return result;
    }

	/**
	 * an XML Parser
	 * @param arg
	 * @return A Tree Object
	 */
	public static Tree build(String arg) {

	  try {
		  File file = new File(arg);
		  DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
		  DocumentBuilder db = dbf.newDocumentBuilder();
		  Document doc = db.parse(file);
		  doc.getDocumentElement().normalize();
		  Node root = doc.getDocumentElement();

		  Tree result = buildup(new Tree(root.getNodeName().split("\n"),0),root);

		  return result;

	  } catch (Exception e) {
	    e.printStackTrace();
	  }

	  return null;
	}
}
