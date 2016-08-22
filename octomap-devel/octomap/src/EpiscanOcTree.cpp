/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */

#include <octomap/EpiscanOcTree.h>

namespace octomap {


    // node implementation  --------------------------------------
    std::ostream& EpiscanOcTreeNode::writeData(std::ostream &s) const {
        s.write((const char*) &value, sizeof(value)); // occupancy
        s.write((const char*) &scan_data, sizeof(ScanData)); // scan_data
        s.write((const char*) &scan_probability, sizeof(float)); // probability
        return s;
    }

    std::istream& EpiscanOcTreeNode::readData(std::istream &s) {        
        s.read((char*) &value, sizeof(value)); // occupancy
        s.read((char*) &scan_data, sizeof(ScanData)); // scan_data
        s.read((char*) &scan_probability, sizeof(float)); // probability
        return s;
    }

    EpiscanOcTreeNode::ScanData EpiscanOcTreeNode::getAverageChildScanData() const {
        int md = 0;
        int mg = 0;
        int mi = 0;
        int c = 0;
    
        if (children != NULL){
            for (int i=0; i<8; i++) {
                EpiscanOcTreeNode* child = static_cast<EpiscanOcTreeNode*>(children[i]);
        
                if (child != NULL && child->isScanDataSet()) {
                    md += child->getScanData().d;
                    mg += child->getScanData().g;
                    mi += child->getScanData().i;
                    ++c;
                }
            }
        }
    
        if (c > 0) {
            md /= c;
            mg /= c;
            mi /= c;
            return ScanData((uint8_t) md, (uint8_t) mg, (uint8_t) mi);
        }
        else { // no child had a scan data other than full values
            return ScanData(255, 255, 255);
        }
    }

    float EpiscanOcTreeNode::getAverageChildScanProbability() const {
        float mp = 0;
        int c = 0;
    
        if (children != NULL){
            for (int i=0; i<8; i++) {
                EpiscanOcTreeNode* child = static_cast<EpiscanOcTreeNode*>(children[i]);
        
                if (child != NULL && child->isScanProbabilitySet()) {
                    mp += child->getScanProbability();
                    ++c;
                }
            }
        }
    
        if (c > 0) {
            mp /= c;
            return mp;
        }
        else { // no child had a scan data other than full values
            return 0.5;
        }
    }


    void EpiscanOcTreeNode::updateScanDataChildren() {      
        scan_data = getAverageChildScanData();
    }

    void EpiscanOcTreeNode::updateScanProbabilityChildren() {      
        scan_probability = getAverageChildScanProbability();
    }

    // tree implementation  --------------------------------------
    EpiscanOcTree::EpiscanOcTree(double resolution)
        : OccupancyOcTreeBase<EpiscanOcTreeNode>(resolution) {
        episcanOcTreeMemberInit.ensureLinking();
    };

    bool EpiscanOcTree::pruneNode(EpiscanOcTreeNode* node) {
        if (!isNodeCollapsible(node)) 
            return false;

        // set value to children's values (all assumed equal)
        node->copyData(*(getNodeChild(node, 0)));
    
        if (node->isScanDataSet()) // TODO check
            node->setScanData(node->getAverageChildScanData());

        if (node->isScanProbabilitySet()) // TODO check
            node->setScanProbability(node->getAverageChildScanProbability());

        // delete children
        for (unsigned int i=0;i<8;i++) {
            deleteNodeChild(node, i);
        }
        delete[] node->children;
        node->children = NULL;

        return true;
    }
  
    bool EpiscanOcTree::isNodeCollapsible(const EpiscanOcTreeNode* node) const{
        // all children must exist, must not have children of
        // their own and have the same occupancy probability
        if (!nodeChildExists(node, 0))
            return false;
    
        const EpiscanOcTreeNode* firstChild = getNodeChild(node, 0);
        if (nodeHasChildren(firstChild))
            return false;

        for (unsigned int i = 1; i<8; i++) {
            // compare nodes only using their occupancy, ignoring color for pruning
            if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i)->getValue() == firstChild->getValue()))
                return false;
        }
    
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// Set Node Data and Prob Funcs /////////////////////////
    ////////////////////////////////////////////////////////////////////////////////



    EpiscanOcTreeNode* EpiscanOcTree::setNodeScanData(const OcTreeKey& key, 
                                                      uint8_t d, 
                                                      uint8_t g, 
                                                      uint8_t i) {
        EpiscanOcTreeNode* n = search (key);
        if (n != 0) {
            n->setScanData(d, g, i); 
        }
        return n;
    }

    EpiscanOcTreeNode* EpiscanOcTree::setNodeScanProbability(const OcTreeKey& key, 
                                                             float p) {
        EpiscanOcTreeNode* n = search (key);
        if (n != 0) {
            n->setScanProbability(p); 
        }
        return n;
    }


    ////////////////////////////////////////////////////////////////////////////////
    /////////////////////// Average Node Data and Prob Funcs ///////////////////////
    ////////////////////////////////////////////////////////////////////////////////

    EpiscanOcTreeNode* EpiscanOcTree::averageNodeScanData(const OcTreeKey& key, 
                                                          uint8_t d, 
                                                          uint8_t g, 
                                                          uint8_t i) {
        EpiscanOcTreeNode* n = search(key);
        if (n != 0) {
            if (n->isScanDataSet()) {
                EpiscanOcTreeNode::ScanData prev_scan_data = n->getScanData();
                n->setScanData((prev_scan_data.d + d)/2,
                               (prev_scan_data.g + g)/2,
                               (prev_scan_data.i + i)/2); 
            }
            else {
                n->setScanData(d, g, i);
            }
        }
        return n;
    }

    EpiscanOcTreeNode* EpiscanOcTree::averageNodeScanProbability(const OcTreeKey& key, 
                                                                 float p) {
        EpiscanOcTreeNode* n = search(key);
        if (n != 0) {
            if (n->isScanProbabilitySet()) {
                float prev_scan_probability = n->getScanProbability();
                n->setScanProbability((prev_scan_probability + p)/2);
            }
            else {
                n->setScanProbability(p);
            }
        }
        return n;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////// Integrate Node Data and Prob Funcs //////////////////////
    ////////////////////////////////////////////////////////////////////////////////

    EpiscanOcTreeNode* EpiscanOcTree::integrateNodeScanData(const OcTreeKey& key, 
                                                            uint8_t d, 
                                                            uint8_t g, 
                                                            uint8_t i) {
        EpiscanOcTreeNode* n = search (key);
        if (n != 0) {
            if (n->isScanDataSet()) {
                EpiscanOcTreeNode::ScanData prev_scan_data = n->getScanData();
                double node_prob = n->getOccupancy();
                uint8_t new_d = (uint8_t) ((double) prev_scan_data.d * node_prob 
                                           +  (double) d * (0.99-node_prob));
                uint8_t new_g = (uint8_t) ((double) prev_scan_data.g * node_prob 
                                           +  (double) g * (0.99-node_prob));
                uint8_t new_i = (uint8_t) ((double) prev_scan_data.i * node_prob 
                                           +  (double) i * (0.99-node_prob));
                n->setScanData(new_d, new_g, new_i); 
            }
            else {
                n->setScanData(d, g, i);
            }
        }
        return n;
    }


    EpiscanOcTreeNode* EpiscanOcTree::integrateNodeScanProbability(const OcTreeKey& key, 
                                                                   float p) {
        EpiscanOcTreeNode* n = search (key);
        if (n != 0) {
            if (n->isScanDataSet()) {
                float prev_scan_probability = n->getScanProbability();
                double node_prob = n->getOccupancy();
                uint8_t new_p = (uint8_t) ((double) prev_scan_probability * node_prob 
                                           +  (double) p * (0.99-node_prob));
                n->setScanProbability(new_p); 
            }
            else {
                n->setScanProbability(p);
            }
        }
        return n;
    }
  
    ////////////////////////////////////////////////////////////////////////////////

    void EpiscanOcTree::updateInnerOccupancy() {
        this->updateInnerOccupancyRecurs(this->root, 0);
    }

    void EpiscanOcTree::updateInnerOccupancyRecurs(EpiscanOcTreeNode* node, unsigned int depth) {
        // only recurse and update for inner nodes:
        if (nodeHasChildren(node)){
            // return early for last level:
            if (depth < this->tree_depth){
                for (unsigned int i=0; i<8; i++) {
                    if (nodeChildExists(node, i)) {
                        updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
                    }
                }
            }
            node->updateOccupancyChildren();
            node->updateScanDataChildren();
            node->updateScanProbabilityChildren();
        }
    }

    void EpiscanOcTree::writeScanDataHistogram(std::string filename) {

#ifdef _MSC_VER
        fprintf(stderr, "The scan data histogram uses gnuplot, this is not supported under windows.\n");
#else
        // build DGI histogram
        std::vector<int> histogram_d (256,0);
        std::vector<int> histogram_g (256,0);
        std::vector<int> histogram_i (256,0);
        for(EpiscanOcTree::tree_iterator it = this->begin_tree(),
                end=this->end_tree(); it!= end; ++it) {
            if (!it.isLeaf() || !this->isNodeOccupied(*it)) continue;
            EpiscanOcTreeNode::ScanData& s = it->getScanData();
            ++histogram_d[s.d];
            ++histogram_g[s.g];
            ++histogram_i[s.i];
        }
        // plot data
        FILE *gui = popen("gnuplot ", "w");
        fprintf(gui, "set term postscript eps enhanced scan data\n");
        fprintf(gui, "set output \"%s\"\n", filename.c_str());
        fprintf(gui, "plot [-1:256] ");
        fprintf(gui, "'-' w filledcurve lt 1 lc 1 tit \"d\",");
        fprintf(gui, "'-' w filledcurve lt 1 lc 2 tit \"g\",");
        fprintf(gui, "'-' w filledcurve lt 1 lc 3 tit \"i\",");
        fprintf(gui, "'-' w l lt 1 lc 1 tit \"\",");
        fprintf(gui, "'-' w l lt 1 lc 2 tit \"\",");
        fprintf(gui, "'-' w l lt 1 lc 3 tit \"\"\n");

        for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_d[i]);    
        fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
        for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_g[i]);    
        fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
        for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_i[i]);    
        fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
        for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_d[i]);    
        fprintf(gui, "e\n");
        for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_g[i]);    
        fprintf(gui, "e\n");
        for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_i[i]);    
        fprintf(gui, "e\n");
        fflush(gui);
#endif
    }

    std::ostream& operator<<(std::ostream& out, EpiscanOcTreeNode::ScanData const& s) {
        return out << '(' << (unsigned int)s.d << ' ' << (unsigned int)s.g << ' ' << (unsigned int)s.i << ')';
    }


    EpiscanOcTree::StaticMemberInitializer EpiscanOcTree::episcanOcTreeMemberInit;

} // end namespace

