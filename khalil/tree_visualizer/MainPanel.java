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

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import javax.swing.JLabel;
import javax.swing.JMenuBar;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.SwingUtilities;

/**
 * paints the visualization of the tree
 * @author andriy
 */
public class MainPanel extends JPanel {
    
    private Dimension size;

    JPanel drawingArea;

    private Font font = new Font("monospaced",Font.PLAIN,10);

    private Tree treetodraw;

    private boolean aal = false;

    private boolean aat = false;

    private int maxlvl = 1;

    private Tree wasclicked = null;

    JScrollPane scroller = null;

    JMenuBar bottom = new JMenuBar();

    JLabel label = new JLabel();

    private boolean vertical = true;

    /**
     * creates the JPanel for JScrollbar and the status line
     * @param tree
     */
    public MainPanel(Tree tree){

        super();

        setOpaque(true);

        size = new Dimension(0,0);
        treetodraw = tree;
        treetodraw.setlvl(maxlvl);
        
        /**
         * the graphical area for the visualisation
         */
        drawingArea = new JPanel() {

            /**
             * sets all sizes and starts drawing functions
             */
            protected void paintComponent(Graphics g){
                //System.out.println(getTreeWidth(g, treetodraw));
                size.width = getTreeWidth(g, treetodraw) + 30;
                size.height = getTreeHeight(g, treetodraw) + 30;
                drawingArea.getBounds().width = getTreeWidth(g, treetodraw);
                drawingArea.getBounds().height = getTreeHeight(g, treetodraw);
                this.setPreferredSize(size);
                this.revalidate();
                
                super.paintComponent(g);
                
                Dimension d = this.getSize();
                int w = getTreeWidth(g,treetodraw);
                int h = getTreeHeight(g, treetodraw);
                if (vertical){
                    drawTree(treetodraw, g, (d.width - w)/2, 30);
                } else {
                    drawTree(treetodraw, g, 30, 30);
                }

            }

            /**
             * draws the root node of a tree in the drawingarea
             */
            public void drawNode(Tree tree,Graphics g, int x, int y) {
                  
                  g.setFont(font) ;
                  FontMetrics fm =  g.getFontMetrics(font) ;
                  if (tree.isLeaf()){
                      g.setColor(Color.GREEN);
                  } /*else if (data == 1){
                      g.setColor(Color.cyan);
                  }*/
                  else g.setColor(Color.lightGray) ;
                  int width = getStringWidth(g,tree.data);
                  int margin = fm.stringWidth(" ") ;
                  int height = fm.getHeight() ;

                  int nodewidth = getNodeWidth(g, tree);
                  int nodeheight = getNodeHeight(g, tree);

                  g.fill3DRect(x,y,nodewidth, nodeheight,true);
                  tree.x = x; tree.y = y; tree.widht = 2*margin + width; tree.heigth = 2*height;
                  g.setColor(Color.black) ;
                  for(int i = 0; i < tree.data.length; i++){
                      Graphics2D g2 = (Graphics2D)g;

                      if(aat){
                        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                                RenderingHints.VALUE_ANTIALIAS_ON);
                      }
                      
                      g.drawString(tree.data[i], x + margin, y + (int)(height*1.3) + (i*height));

                      if(aat){
                        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                                RenderingHints.VALUE_ANTIALIAS_OFF);
                      }
                  }
            }

            /**
             * calculates the coordinates of each node and draws the tree
             */
            public void drawTree(Tree tree,Graphics g, int x, int y) {

                  int nw = getNodeWidth(g,tree) ;
                  int tw = getTreeWidth(g,tree) ;
                  int th = getTreeHeight(g, tree);
                  int nh = getNodeHeight(g,tree) ;
                  
                  if(vertical){
                      drawNode(tree,g,x + (tw - nw)/2,y);
                  } else {
                      drawNode(tree,g,x,y);
                  }
                  
                  if(tree.draw){
                      if (tree.children.size() > 0) {
                          
                          int dx = x ; // start at left
                          int dy = y ;

                          for(int k = 0 ; k < tree.children.size() ; k++) {
                              Tree t = (Tree)(tree.children.elementAt(k)) ;
                              int cw = getTreeWidth(g,t);
                              int ch = getTreeHeight(g,t);
                              // draw the edge
                              g.setColor(Color.darkGray);

                              Graphics2D g2 = (Graphics2D)g;

                              if(aal){
                                g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                                        RenderingHints.VALUE_ANTIALIAS_ON);
                              }
                              if(vertical){
                                  g.drawLine(x+tw/2,y+nh, dx+cw/2,y+2*nh - 1) ;
                              } else {
                                  
                              }
                              
                              if(aal){
                                g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                                        RenderingHints.VALUE_ANTIALIAS_OFF);
                              }
                              

                              // draw child tree ...
                              if (vertical){
                                  drawTree(t,g,dx,y+2*nh) ;
                              } else {
                                  drawTree(t,g,x + getLevelWidth(g, treetodraw, tree.depth-1, 0),dy);
                              }
                              
                              // move over to right on drawing pane
                              if (vertical){
                                  dx += cw ;
                              } else {
                                  dy += ch ;
                              }
                              
                          }
                      }
                  }
               }

            /**
             * calculates the width of the whole level of a tree
             */
            public int getLevelWidth(Graphics g, Tree tree, int lvl, int n){

                if(tree.depth == lvl && n < getNodeWidth(g, tree)){
                    n = getNodeWidth(g, tree);
                } else {
                    if(!tree.isLeaf()){
                        for(int i = 0; i < tree.children.size(); i++){
                            Tree t = (Tree)tree.children.elementAt(i);
                            n = getLevelWidth(g, t,lvl,n);
                        }
                    }
                }

                return n;
            }
            
            /**
             * returns the whole width of a visual tree
             */
            public int getTreeWidth(Graphics g, Tree tree) {
                int w = (vertical) ? getNodeWidth(g, tree) + 30
                        : getNodeWidth(g, tree);
                int wc = 0 ;
                if ((!tree.isLeaf() && tree.draw)){
                    if (vertical) {
                        for(int i = 0 ; i < tree.children.size() ; i++){
                            Tree t = (Tree)tree.children.elementAt(i);
                            wc += getTreeWidth(g,t);
                        }
                    } else {
                        Tree largest = (Tree)tree.children.elementAt(0);
                        for (int i = 0; i < tree.children.size(); i++){
                            Tree prove = (Tree)tree.children.elementAt(i);
                            if(getTreeWidth(g, prove) > getTreeWidth(g, largest)){
                                largest = prove;
                            }
                        }
                        wc += w + getTreeWidth(g, largest);
                    }
                }
                return Math.max(w,wc) ;
            }

            /**
             * returns the width of a node
             */
            public int getNodeHeight(Graphics g,Tree tree) {
              // get the height of this node
              g.setFont(font) ;
              FontMetrics fm =  g.getFontMetrics(font) ;
              int h = fm.getHeight() ;
              int hs = this.getStringHeight(g, tree.data) ;
              return hs+h;
            }

            /**
             * returns the height of a multiline string
             */
            public int getStringHeight(Graphics g, String[] s){
               g.setFont(font) ;
               FontMetrics fm =  g.getFontMetrics(font) ;
               int h = fm.getHeight();
               if(s.length > 0){
                   return h*s.length;
               } else {
                   return h;
               }
            }

            /**
             * returns the width of a node
             */
            public int getNodeWidth(Graphics g, Tree tree) {
              // get the width of this node
              g.setFont(font) ;
              FontMetrics fm =  g.getFontMetrics(font) ;
              int w = getStringWidth(g,tree.data);
              int margin = fm.stringWidth(" ") ;
              return 2*margin + w;
            }

            /**
             * returns the width of a multiline string
             */
            public int getStringWidth(Graphics g,String[] s){
               g.setFont(font) ;
               FontMetrics fm =  g.getFontMetrics(font) ;
               int w = fm.stringWidth(s[0]);
                  for(int i = 0; i < s.length; i++){
                      if(w < fm.stringWidth(s[i])){
                          w = fm.stringWidth(s[i]);
                      }
                  }
                  return w;
            }

            /**
             * returns the height of a visual tree
             */
            public int getTreeHeight(Graphics g,Tree tree) {
              int h = (vertical) ? getNodeHeight(g,tree)+30
                      : getNodeHeight(g, tree)+10;

              if (!tree.isLeaf() && tree.draw) {
                 int h1 = h;
                 for(int i = 1 ; i < tree.children.size() ; i++) {
                    Tree t2 = ((Tree)tree.children.elementAt(i));
                    int h2 = getTreeHeight(g,t2) ;
                    if(vertical){
                        if (h2 > h1) h1 = h2 ;
                    } else {
                        h1 += h2;
                    }
                 }
                 h = Math.max(h, h1);
              }
              
              return h;
            }
            
        };
        
        drawingArea.setBackground(Color.white);
        drawingArea.addMouseListener(new MyMouseListener());

        //bottom menu
        

        info();
               
        bottom.add(label);
        //bottom menu

        scroller = new JScrollPane(drawingArea);
        scroller.setPreferredSize(new Dimension(640,480));
        setLayout(new BorderLayout());
        add(scroller,BorderLayout.CENTER);
        add(bottom,BorderLayout.SOUTH);

        
    }

    /**
     * switches the line antialiasing on/off
     */
    public void setAAl(){
        aal = !aal;
        this.revalidate();
        info();
        drawingArea.repaint();
    }

    /**
     * switches the text antialiasing on/off
     */
    public void setAAt(){
        aat = !aat;
        this.revalidate();
        info();
        drawingArea.repaint();
    }

    /**
     * sets the maximal global level to paint
     * @param i
     */
    public void setMaxlevel(int i){
        treetodraw.setlvl(i);
        maxlvl = i;
        this.revalidate();
        info();
        drawingArea.repaint();
    }

    /**
     * changes the aligment of the painted content horizontal/vertical
     */
    public void changeAligment(){
        vertical = !vertical;
        this.revalidate();
        info();
        drawingArea.repaint();
    }

    /**
     * changes the zoom factor
     * @param i zoom factor (font size)
     */
    public void setZoom(int i){
        font = new Font("monospaced",Font.PLAIN,i);
        this.revalidate();
        info();
        drawingArea.repaint();
    }

    /**
     * revalidates the content of the status line
     */
    public void info(){
        label.setText("Info: Custom level: " + maxlvl + "/" + 
                (treetodraw.getMaxLevel(0)+1) + "   Antialiasing:   lines: " +
                aal + "  text: " + aat +
                "   Zoom: " + font.getSize());
    }

    /**
     * listens to the mouse events
     */
    class MyMouseListener extends MouseAdapter implements MouseListener {

        /**
         * graps the mouse position on releasing a mousebutton and proves if a
         * node was clicked
         *
         * @param e MouseEvent captured by the MouseListener
         */
        public void mouseReleased(MouseEvent e) {

            boolean changed = false;
            if (SwingUtilities.isLeftMouseButton(e)){
                int x = e.getX();
                int y = e.getY();

                whichwasclicked(treetodraw, x, y);

                changed = true;
            }
            if (changed){
                drawingArea.setPreferredSize(size);
                drawingArea.revalidate();
            }
            drawingArea.repaint();
            wasclicked = null;
        }

        /**
         * calculation of the clicked node
         * @param tree the whole tree that has to be prooved
         * @param x the x coordinate of the mouse on click
         * @param y the y coordinate of the mouse on click
         */
        public void whichwasclicked(Tree tree, int x, int y){
            if(wasclicked == null){
                if(tree.wasClicked(x, y)){
                    tree.draw = !tree.draw;
                } else {
                    if(tree.children.size() > 0){
                        for(int i = 0; i < tree.children.size(); i++){
                            whichwasclicked((Tree) tree.children.get(i), x, y);
                        }
                    }
                }
            }
        }
    }

}
