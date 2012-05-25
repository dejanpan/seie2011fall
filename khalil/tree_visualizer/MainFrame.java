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
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.File;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

/**
 * Creates and handels the main Frame
 * @author andriy
 */
public class MainFrame extends JFrame implements ActionListener, KeyListener, ChangeListener{

    public MainPanel mainpanel;

    private Tree tree;

    JComboBox lvls = new JComboBox();

    JSlider slider;

    /**
     * opens a new FileChooser for selecting the xml file.
     */
    public MainFrame(){
        JFileChooser fc = new JFileChooser(System.getProperty("user.dir"));

            int val = fc.showOpenDialog(null);

            if(val == JFileChooser.APPROVE_OPTION){
                File file = fc.getSelectedFile();
                Tree tree2 = Parser.build(file.getPath());
                new MainFrame(tree2);
            } else {
                System.exit(0);
            }
    }


    /**
     * Creates a new JFrame which contains all contenses. Adds a new menubar
     * with all the buttons and panels to it.
     * 
     * @param tree the Tree object has t be visualized
     */
    public MainFrame(Tree tree){
        super("Visual XML");
        this.tree = tree;
        //System.out.println(tree.toString());

        //top menu
        JMenuBar mb = getmenubar();
        JMenu file = getmenu("File");
        JMenuItem open = getmenuitem("Open");
        open.setMnemonic(KeyEvent.VK_O);
        file.add(open);
        file.add(getmenuitem("Exit"));
        mb.add(file);

        JMenu expand = getmenu("Expand");
        expand.add(getmenuitem("Expand all"));
        expand.add(getmenuitem("Collapse all"));
        mb.add(expand);

        JMenu options = getmenu("Antialiasing");
        options.add(getcheckbox("Lines"));
        options.add(getcheckbox("Text"));
        mb.add(options);

        JButton aligm = new JButton("Aligment");
        aligm.addActionListener(this);
        //mb.add(aligm);

        this.setJMenuBar(mb);

        mb.add(new JLabel(" Max lvl: "));
        lvls.addActionListener(this);
        for(int i = 1; i <= tree.getMaxLevel(0) + 1; i++){
            lvls.addItem(i);
        }
        lvls.setSelectedIndex(0);
        mb.add(lvls);
        mb.add(new JLabel(" Zoom: "));
        slider = new JSlider(1, 30, 10);
        slider.addChangeListener(this);
        slider.setMajorTickSpacing(1);
        slider.setPaintTicks(true);

        mb.add(slider);
        //end top menu

        

        //this.addKeyListener(this);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        mainpanel = new MainPanel(tree);
        this.setLayout(new BorderLayout());
        add(mainpanel,BorderLayout.CENTER);
        pack();
        setVisible(true);
    }


    /**
     * creates a new JCheckBoxr and add an ActionListener to this JCheckBox
     * @param s Name and Text of the CheckBox
     * @return new JCheckBox
     */
    public JCheckBox getcheckbox(String s){
        JCheckBox result = new JCheckBox();
        result.setName(s);
        result.setText(s);
        result.addActionListener(this);

        return result;
    }

    /**
     * creates a new JMenuBar and add an ActionListener to this JMenuBar
     * @param s Name and Text of the Menubar
     * @return new JMenubar
     */
    public JMenuBar getmenubar(){
        JMenuBar result = new JMenuBar();

        return result;
    }

    /**
     * creates a new JMenu and add an ActionListener to this JMenu
     * @param s Name and Text of the Menu
     * @return new JMenu
     */
    public JMenu getmenu(String s){
        JMenu result = new JMenu();
        result.setText(s);
        result.setName(s);
        result.setArmed(false);

        result.addActionListener(this);

        return result;
    }

    /**
     * creates a new JMenuItem and add an ActionListener to this JMenuItem
     * @param s Name and Text of the MenuItem
     * @return new JMenuItem
     */
    public JMenuItem getmenuitem(String s){
        JMenuItem result = new JMenuItem();
        result.setName(s);
        result.setText(s);

        result.addActionListener(this);

        return result;
    }

    /**
     * Waits for an action of the GUI and reacts on them
     *
     * @param e the ActionEvent captured by the ActionListener
     */
    public void actionPerformed(ActionEvent e) {

        String cmd = e.getActionCommand();

        if(cmd.equals("Exit")){
            System.exit(0);
        } else if (cmd.equals("Open")){
            JFileChooser fc = new JFileChooser();
            int val = fc.showOpenDialog(null);

            if(val == JFileChooser.APPROVE_OPTION){
                File file = fc.getSelectedFile();
                Tree treef = Parser.build(file.getPath());
                new MainFrame(treef);
                this.dispose();
            }
        } else if(cmd.equals("Lines")){
            mainpanel.setAAl();
        } else if(cmd.equals("Text")){
            mainpanel.setAAt();
        } else if(cmd.equals("Expand all")){
            mainpanel.setMaxlevel(tree.getMaxLevel(0) + 1);
        } else if(cmd.equals("Collapse all")){
            mainpanel.setMaxlevel(1);
        } else if(cmd.equals("comboBoxChanged")){
            if(e.getModifiers() == 16){
                mainpanel.setMaxlevel(lvls.getSelectedIndex()+1);
            }
        } else if(cmd.equals("Aligment")){
            mainpanel.changeAligment();
        } else {
            System.err.println(cmd + " not supported yet.");
        }

    }

    public void keyTyped(KeyEvent e) {
        
    }

    public void keyPressed(KeyEvent e) {
        //throw new UnsupportedOperationException("Not supported yet.");
    }

    public void keyReleased(KeyEvent e) {
        //throw new UnsupportedOperationException("Not supported yet.");
    }

    /**
     * Listens to the changes of the zoom bar
     * @param arg0 Event captured by the ChangeListener
     */
    public void stateChanged(ChangeEvent arg0) {
        mainpanel.setZoom((int)slider.getValue());
    }

}
