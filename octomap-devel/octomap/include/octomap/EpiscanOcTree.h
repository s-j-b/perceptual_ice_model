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

#ifndef OCTOMAP_EPISCAN_OCTREE_H
#define OCTOMAP_EPISCAN_OCTREE_H


#include <iostream>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

namespace octomap {
  
    // forward declaraton for "friend"
    class EpiscanOcTree;
  
    // node definition
    class EpiscanOcTreeNode : public OcTreeNode {    
    public:
        friend class EpiscanOcTree; // needs access to node children (inherited)
    
        class ScanData {
        public:
        ScanData() : d((uint8_t)255), g((uint8_t)255), i((uint8_t)255) {}
        ScanData(uint8_t _d, uint8_t _g, uint8_t _i) 
            : d(_d), g(_g), i(_i) {}
            inline bool operator== (const ScanData &other) const {
                return (d==other.d && g==other.g && i==other.i);
            }
            inline bool operator!= (const ScanData &other) const {
                return (d!=other.d || g!=other.g || i!=other.i);
            }
            uint8_t d, g, i;
        };

        /*
        class ScanInfo {
        public:
        ScanInfo() : s(255, 255, 255), p(0.5) {}
        ScanInfo(EpiscanOcTreeNode::ScanData _s, float _p)
            : s(_s), p(_p) {}
            inline bool operator== (const ScanInfo &other) const {
                return (s==other.s && p==other.p);
            }
            inline bool operator!= (const ScanInfo &other) const {
                return (s!=other.s || p!=other.p);
            }
            ScanData s;
            float p;
        };
        */

    public:
    EpiscanOcTreeNode() : OcTreeNode() {}

    EpiscanOcTreeNode(const EpiscanOcTreeNode& rhs) :
        OcTreeNode(rhs),
            scan_data(rhs.scan_data),
            scan_probability(rhs.scan_probability) {}

        bool operator==(const EpiscanOcTreeNode& rhs) const{
            return (rhs.value == value && rhs.scan_data == scan_data && rhs.scan_probability == scan_probability);
        }

        void copyData(const EpiscanOcTreeNode& from){
            OcTreeNode::copyData(from);
            this->scan_data =  from.getScanData();
            this->scan_probability =  from.getScanProbability();
        }
        
        inline ScanData getScanData() const { return scan_data; }
        inline void  setScanData(ScanData s) {this->scan_data = s; }
        inline void  setScanData(uint8_t d, uint8_t g, uint8_t i) {
            this->scan_data = ScanData(d, g, i); 
        }

        ScanData& getScanData() { return scan_data; }


        inline float getScanProbability() const { return scan_probability; }
        inline void  setScanProbability(float p) {this->scan_probability = p; }

        float& getScanProbability() { return scan_probability; }

        // has any scan_data been integrated? (exact init conditions is very unlikely...)
        inline bool isScanDataSet() const { 
            return ((scan_data.d != 255) || (scan_data.g != 255) || (scan_data.i != 255)); 
        }

        // has any scan_data been integrated? (exact init conditions is very unlikely...)
        inline bool isScanProbabilitySet() const { 
            return (scan_probability != 0.5);
        }

        EpiscanOcTreeNode::ScanData getAverageChildScanData() const;
        float getAverageChildScanProbability() const;

        void updateScanDataChildren();
        void updateScanProbabilityChildren();


        // file I/O
        std::istream& readData(std::istream &s);
        std::ostream& writeData(std::ostream &s) const;
    
    protected:
        ScanData scan_data;
        float scan_probability;
    };

    // tree definition
    class EpiscanOcTree : public OccupancyOcTreeBase <EpiscanOcTreeNode> {

    public:
        /// Default constructor, sets resolution of leafs
        EpiscanOcTree(double resolution);
      
        /// virtual constructor: creates a new object of same type
        /// (Covariant return type requires an up-to-date compiler)
        EpiscanOcTree* create() const {return new EpiscanOcTree(resolution); }

        std::string getTreeType() const {return "EpiscanOcTree";}
    
        /**
         * Prunes a node when it is collapsible. This overloaded
         * version only considers the node occupancy for pruning,
         * different scan info of child nodes are ignored.
         * @return true if pruning was successful
         */
        virtual bool pruneNode(EpiscanOcTreeNode* node);
    
        virtual bool isNodeCollapsible(const EpiscanOcTreeNode* node) const;
       
        ////////// NOTE: "ScanInfo" means both "ScanData" + "ScanProbability" //////////

        ////////////////////////////////////////////////////////////////////////////////
        ///////////////////// Set Node Contents (Data, Prob, Info) /////////////////////
        ////////////////////////////////////////////////////////////////////////////////

        
        // set node scan data at given key or coordinate. Replaces previous scan data.
        EpiscanOcTreeNode* setNodeScanData(const OcTreeKey& key,
                                           uint8_t d, uint8_t g, uint8_t i);
        EpiscanOcTreeNode* setNodeScanData(float x, float y, float z,
                                           uint8_t d, uint8_t g, uint8_t i) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
            return setNodeScanData(key,d,g,i);
        }


        // set node scan probability at given key or coordinate. Replaces previous scan probability.
        EpiscanOcTreeNode* setNodeScanProbability(const OcTreeKey& key, float p);
        EpiscanOcTreeNode* setNodeScanProbability(float x, float y, float z, float p) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
            return setNodeScanProbability(key,p);
        }

        // set node scan info at given key or coordinate. Replaces previous scan info.
        EpiscanOcTreeNode* setNodeScanInfo(const OcTreeKey& key,
                                           uint8_t d, uint8_t g, uint8_t i, float p) {
            setNodeScanData(key,d,g,i);
            return setNodeScanProbability(key,p);
            
        }
        EpiscanOcTreeNode* setNodeScanInfo(float x, float y, float z,
                                           uint8_t d, uint8_t g, uint8_t i, float p) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
            return setNodeScanInfo(key,d,g,i,p);
        }


        ////////////////////////////////////////////////////////////////////////////////
        /////////////////// Average Node Contents (Data, Prob, Info) ///////////////////
        ////////////////////////////////////////////////////////////////////////////////


        
        // average node scan data at given key or coordinate. Replaces previous scan data.
        EpiscanOcTreeNode* averageNodeScanData(const OcTreeKey& key,
                                               uint8_t d, uint8_t g, uint8_t i);
        EpiscanOcTreeNode* averageNodeScanData(float x, float y, float z,
                                               uint8_t d, uint8_t g, uint8_t i) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
            return averageNodeScanData(key,d,g,i);
        }


        // average node scan probability at given key or coordinate. Replaces previous scan probability.
        EpiscanOcTreeNode* averageNodeScanProbability(const OcTreeKey& key, float p);
        EpiscanOcTreeNode* averageNodeScanProbability(float x, float y, float z, float p) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
            return averageNodeScanProbability(key,p);
        }

        // average node scan info at given key or coordinate. Replaces previous scan info.
        EpiscanOcTreeNode* averageNodeScanInfo(const OcTreeKey& key,
                                               uint8_t d, uint8_t g, uint8_t i, float p) {
            averageNodeScanData(key,d,g,i);
            return averageNodeScanProbability(key,p);
            
        }
        EpiscanOcTreeNode* averageNodeScanInfo(float x, float y, float z,
                                               uint8_t d, uint8_t g, uint8_t i, float p) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
            return averageNodeScanInfo(key,d,g,i,p);
        }

        ////////////////////////////////////////////////////////////////////////////////
        ////////////////// Integrate Node Contents (Data, Prob, Info) //////////////////
        ////////////////////////////////////////////////////////////////////////////////

        
        // integrate node scan data at given key or coordinate. Replaces previous scan data.
        EpiscanOcTreeNode* integrateNodeScanData(const OcTreeKey& key,
                                                 uint8_t d, uint8_t g, uint8_t i);
        EpiscanOcTreeNode* integrateNodeScanData(float x, float y, float z,
                                                 uint8_t d, uint8_t g, uint8_t i) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
            return integrateNodeScanData(key,d,g,i);
        }


        // integrate node scan probability at given key or coordinate. Replaces previous scan probability.
        EpiscanOcTreeNode* integrateNodeScanProbability(const OcTreeKey& key, float p);
        EpiscanOcTreeNode* integrateNodeScanProbability(float x, float y, float z, float p) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
            return integrateNodeScanProbability(key,p);
        }

        // integrate node scan info at given key or coordinate. Replaces previous scan info.
        EpiscanOcTreeNode* integrateNodeScanInfo(const OcTreeKey& key,
                                                 uint8_t d, uint8_t g, uint8_t i, float p) {
            integrateNodeScanData(key,d,g,i);
            return integrateNodeScanProbability(key,p);
            
        }
        EpiscanOcTreeNode* integrateNodeScanInfo(float x, float y, float z,
                                                 uint8_t d, uint8_t g, uint8_t i, float p) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
            return integrateNodeScanInfo(key,d,g,i,p);
        }

        // update inner nodes, sets scan info to average child scan info
        void updateInnerOccupancy();

        // uses gnuplot to plot a DGI histogram in EPS format
        void writeScanDataHistogram(std::string filename);
    
    protected:
        void updateInnerOccupancyRecurs(EpiscanOcTreeNode* node, unsigned int depth);

        /**
         * Static member object which ensures that this OcTree's prototype
         * ends up in the classIDMapping only once. You need this as a 
         * static member in any derived octree class in order to read .ot
         * files through the AbstractOcTree factory. You should also call
         * ensureLinking() once from the constructor.
         */
        class StaticMemberInitializer{
        public:
            StaticMemberInitializer() {
                EpiscanOcTree* tree = new EpiscanOcTree(0.1);
                tree->clearKeyRays();
                AbstractOcTree::registerTreeType(tree);
            }

            /**
             * Dummy function to ensure that MSVC does not drop the
             * StaticMemberInitializer, causing this tree failing to register.
             * Needs to be called from the constructor of this octree.
             */
            void ensureLinking() {};
        };
        /// static member to ensure static initialization (only once)
        static StaticMemberInitializer episcanOcTreeMemberInit;

    };

    //! user friendly output in format (d g i p)
    std::ostream& operator<<(std::ostream& out, EpiscanOcTreeNode::ScanData const& s);

} // end namespace

#endif
