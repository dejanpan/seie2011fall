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

import java.util.Vector;

public class Tree {
    String[] data;
    int depth;
    Vector children = new Vector();
    boolean draw = true;
    int x,y,widht,heigth;


    Tree(String[] data, int i) {
        this.data = data;
        this.depth = i;
    }

    /**
     * adds a child to the tree
     * @param tree
     */
    void addChild(Tree tree) {
        if(tree != null){
            children.addElement(tree);
        }
    }

    /**
     * checks if the mouse coordinates are in the rect of this tree's root node
     * @param x
     * @param y
     * @return
     */
    public boolean wasClicked(int x, int y){
	   return (x < this.x + this.widht) && (x > this.x) && (y < this.y + this.heigth) && (y > this.y);
    }

    @Override
    public String toString() {
        return toString(0) ;
    }

    /**
     * sets the maximal level to paint. changes the draw atribute to true/false
     * @param n
     */
    public void setlvl(int n){
        if(this.depth > n){
            this.draw = false;
        } else {
            this.draw = true;
        }
        
        if (!this.isLeaf()) {
		   for(int i = 0; i < children.size(); i++){
			   ((Tree)children.elementAt(i)).setlvl(n);
		   }
        }
    }

    /**
     * returns the string of this Tree object
     * @param tab
     * @return
     */
    public String toString(int tab) {
        String result = String.valueOf(depth) ;
        // calculate (String) tabs first
        for(int i = 0 ; i <= tab-1 ; i ++) result += "  " ;
        result += "|-" + data[0] + "\n" ;
        for(int j = 0 ; j <= children.size()-1 ; j++)
            result += ((Tree)(children.elementAt(j))).toString(tab+1) ;
        return result;
    }

    public int getMaxLevel(int n){
	   if(this.isLeaf()){
		   return Math.max(n, depth);
	   } else {
		   int result = n;
		   for(int i = 0; i < children.size(); i++){
			   Tree t = (Tree)children.elementAt(i);
			   result = t.getMaxLevel(result);
		   }
		   return result;
	   }
    }

    /**
     * checks if this tree has children trees
     * @return
     */
    public boolean isLeaf() {
      return (children.size() == 0) ;
    }
}
