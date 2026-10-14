#include <signal_path/signal_path.h>
#include <gui/gui.h>
#include <gui/tuner.h>
#include <string>

namespace tuner {


    void centerTuning(std::string vfoName, double freq) {
        if (vfoName != "") {
            if (gui::waterfall.vfos.find(vfoName) == gui::waterfall.vfos.end()) { return; }
            sigpath::vfoManager.setOffset(vfoName, 0);
        }
        double BW = gui::waterfall.getBandwidth();
        double viewBW = gui::waterfall.getViewBandwidth();
        gui::waterfall.setViewOffset((BW / 2.0) - (viewBW / 2.0));
        gui::waterfall.setCenterFrequency(freq);
        gui::waterfall.setViewOffset(0);
        gui::freqSelect.setFrequency(freq);
        sigpath::sourceManager.tune(freq);
    }

    void normalTuning(std::string vfoName, double freq) {
        // If no VFO name is provided, use centerTuning instead
        if (vfoName == "") {
            centerTuning(vfoName, freq);
            return;
        }
        
        // Check if VFO exists
        if (gui::waterfall.vfos.find(vfoName) == gui::waterfall.vfos.end()) {
            return;
        }
    
        // Get the VFO object
        ImGui::WaterfallVFO* vfo = gui::waterfall.vfos[vfoName];
        
        // Get current waterfall view properties
        double viewBW = gui::waterfall.getViewBandwidth();
        double BW = gui::waterfall.getBandwidth();
        double currentCenterFreq = gui::waterfall.getCenterFrequency();
		// Get usable Spectrum percentage for current source
		double usableSpectrumRatio = gui::waterfall.getUsableSpectrumRatio();
		// Calculate actual usable bandwidth and its boundaries
		double usableBW = (BW * usableSpectrumRatio);
        double unusableBWHalf = (BW - usableBW) / 2.0;
        
        // Define boundaries of usable bandwidth (relative to center frequency)
        double usableBottom = -usableBW / 2.0;
        double usableTop = usableBW / 2.0;
        
        // Calculate current VFO position and desired new frequency
        double currentVFOOffset = vfo->centerOffset;
        double currentTuneFreq = currentCenterFreq + vfo->generalOffset;
        double deltaFreq = freq - currentTuneFreq;
        
        // Calculate VFO bandwidth
        double vfoBW = vfo->bandwidth;
        
        // Calculate the proposed new VFO position
        double newVFOPos = currentVFOOffset + deltaFreq;
        double newVFOBottom = newVFOPos - (vfoBW / 2.0);
        double newVFOTop = newVFOPos + (vfoBW / 2.0);
        
        // Case 1: Requested position would place VFO within usable bandwidth
        if (newVFOBottom >= usableBottom && newVFOTop <= usableTop) {
            // Check if the VFO would be visible in the current view
            double viewOffset = gui::waterfall.getViewOffset();
            double viewBottom = viewOffset - (viewBW / 2.0);
            double viewTop = viewOffset + (viewBW / 2.0);
            
            if (newVFOBottom >= viewBottom && newVFOTop <= viewTop) {
                // VFO is within current view, just update the offset
                sigpath::vfoManager.setCenterOffset(vfoName, newVFOPos);
            } else {
                // VFO would be outside current view, adjust view to include it
                double newViewOffset = newVFOPos; // Center view on VFO
                
                // Ensure view is within usable bandwidth
                if (newViewOffset - (viewBW / 2.0) < usableBottom) {
                    newViewOffset = usableBottom + (viewBW / 2.0);
                } else if (newViewOffset + (viewBW / 2.0) > usableTop) {
                    newViewOffset = usableTop - (viewBW / 2.0);
                }
                
                gui::waterfall.setViewOffset(newViewOffset);
                sigpath::vfoManager.setCenterOffset(vfoName, newVFOPos);
            }
            return;
        }
        
        // Case 2: Requested position would place VFO outside usable bandwidth
        // We need to adjust by retuning the SDR
        
        // Calculate new SDR center frequency and VFO offset
        double newVFOOffset;
        
        if (deltaFreq < 0) {
            // Moving to a lower frequency
            // Place VFO one VFO bandwidth away from the upper edge of usable bandwidth
            newVFOOffset = usableTop - (vfoBW * 1.5);
            
            // Ensure the VFO is still within usable bandwidth
            if (newVFOOffset - (vfoBW / 2.0) < usableBottom) {
                newVFOOffset = usableBottom + (vfoBW / 2.0);
            }
        } else {
            // Moving to a higher frequency
            // Place VFO one VFO bandwidth away from the lower edge of usable bandwidth
            newVFOOffset = usableBottom + (vfoBW * 1.5);
            
            // Ensure the VFO is still within usable bandwidth
            if (newVFOOffset + (vfoBW / 2.0) > usableTop) {
                newVFOOffset = usableTop - (vfoBW / 2.0);
            }
        }
        
        // Calculate new SDR center frequency based on target frequency and new VFO offset
        double newSDRFreq = freq - newVFOOffset;
        
        // Update SDR tuning and VFO position
        sigpath::vfoManager.setOffset(vfoName, newVFOOffset);
        gui::waterfall.setCenterFrequency(newSDRFreq);
        sigpath::sourceManager.tune(newSDRFreq);
        
        // Adjust view to ensure the VFO is visible
        // If view bandwidth is greater than or equal to usable bandwidth, center the view
        if (viewBW >= usableBW) {
            gui::waterfall.setViewOffset(0.0);
        } 
        // Otherwise, position view to show the VFO
        else {
            double viewHalfBW = viewBW / 2.0;
            
            if (newVFOOffset < 0) {
                // If VFO is in the lower half of usable bandwidth
                gui::waterfall.setViewOffset(usableBottom + viewHalfBW);
            } else {
                // If VFO is in the upper half of usable bandwidth
                gui::waterfall.setViewOffset(usableTop - viewHalfBW);
            }
        }
    }    

    void iqTuning(double freq) {
        gui::waterfall.setCenterFrequency(freq);
        gui::waterfall.centerFreqMoved = true;
        sigpath::sourceManager.tune(freq);
    }

    void tune(int mode, std::string vfoName, double freq) {
        if (vfoName == "_current") {
            vfoName = gui::waterfall.selectedVFO;
        }
        switch (mode) {
        case TUNER_MODE_CENTER:
            centerTuning(vfoName, freq);
            break;
        case TUNER_MODE_NORMAL:
            normalTuning(vfoName, freq);
            break;
        case TUNER_MODE_LOWER_HALF:
            normalTuning(vfoName, freq);
            break;
        case TUNER_MODE_UPPER_HALF:
            normalTuning(vfoName, freq);
            break;
        case TUNER_MODE_IQ_ONLY:
            iqTuning(freq);
            break;
        }
    }
}
