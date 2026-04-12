#!/usr/bin/env python3
# This file is a modified version of the original code from: https://github.com/PRBonn/lidar-bonnetal

import importlib.util
import os
import torch
import torch.nn as nn
import torch.nn.functional as F
from CRF import CRF
from ament_index_python.packages import get_package_share_directory


def _load_source(module_name, file_path):
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Segmentator(nn.Module):
    def __init__(self, ARCH, nclasses, path=None, path_append="", strict=False):
        super().__init__()
        self.ARCH = ARCH
        self.nclasses = nclasses
        self.path = path
        self.path_append = path_append
        self.strict = False

        # In ROS2 the package's installed Python files live under
        # share/<pkg>/.../scripts and lib/<pkg>/. The backbone/decoder support
        # modules are installed under lib/<pkg>/backbone and lib/<pkg>/decoder
        # by this package's CMakeLists.txt — but ament_index returns the share
        # directory. We resolve the install prefix from there.
        share_dir = get_package_share_directory('scan2shape_launch')
        install_prefix = os.path.dirname(os.path.dirname(share_dir))
        backbone_decoder_path = os.path.join(
            install_prefix, "lib", "scan2shape_launch") + os.sep

        # get the model
        bboneModule = _load_source(
            "bboneModule", backbone_decoder_path + "backbone/"+self.ARCH["backbone"]["name"]+".py")
        self.backbone = bboneModule.Backbone(params=self.ARCH["backbone"])

        # do a pass of the backbone to initialize the skip connections
        stub = torch.zeros((1,
                            self.backbone.get_input_depth(),
                            self.ARCH["dataset"]["sensor"]["img_prop"]["height"],
                            self.ARCH["dataset"]["sensor"]["img_prop"]["width"]))

        if torch.cuda.is_available():
            stub = stub.cuda()
            self.backbone.cuda()
        _, stub_skips = self.backbone(stub)

        decoderModule = _load_source(
            "decoderModule", backbone_decoder_path + "decoder/"+self.ARCH["backbone"]["name"]+".py")
        self.decoder = decoderModule.Decoder(params=self.ARCH["decoder"],
                                             stub_skips=stub_skips,
                                             OS=self.ARCH["backbone"]["OS"],
                                             feature_depth=self.backbone.get_last_depth())

        self.head = nn.Sequential(nn.Dropout2d(p=ARCH["head"]["dropout"]),
                                  nn.Conv2d(self.decoder.get_last_depth(),
                                            self.nclasses, kernel_size=3,
                                            stride=1, padding=1))

        if self.ARCH["post"]["CRF"]["use"]:
            self.CRF = CRF(self.ARCH["post"]["CRF"]["params"], self.nclasses)
        else:
            self.CRF = None

        # train backbone?
        if not self.ARCH["backbone"]["train"]:
            for w in self.backbone.parameters():
                w.requires_grad = False

        # train decoder?
        if not self.ARCH["decoder"]["train"]:
            for w in self.decoder.parameters():
                w.requires_grad = False

        # train head?
        if not self.ARCH["head"]["train"]:
            for w in self.head.parameters():
                w.requires_grad = False

        # train CRF?
        if self.CRF and not self.ARCH["post"]["CRF"]["train"]:
            for w in self.CRF.parameters():
                w.requires_grad = False

        # print number of parameters and the ones requiring gradients
        weights_total = sum(p.numel() for p in self.parameters())
        weights_grad = sum(p.numel()
                           for p in self.parameters() if p.requires_grad)
        print("Total number of parameters: ", weights_total)
        print("Total number of parameters requires_grad: ", weights_grad)

        # breakdown by layer
        weights_enc = sum(p.numel() for p in self.backbone.parameters())
        weights_dec = sum(p.numel() for p in self.decoder.parameters())
        weights_head = sum(p.numel() for p in self.head.parameters())
        print("Param encoder ", weights_enc)
        print("Param decoder ", weights_dec)
        print("Param head ", weights_head)
        if self.CRF:
            weights_crf = sum(p.numel() for p in self.CRF.parameters())
            print("Param CRF ", weights_crf)

        # get weights
        if path is not None:
            # try backbone
            try:
                w_dict = torch.load(path + "/backbone" + path_append,
                                    map_location=lambda storage, loc: storage)
                # TODO(ankit): Figure out what strict does
                self.backbone.load_state_dict(w_dict, strict=True)
                print("Successfully loaded model backbone weights")
            except Exception as e:
                print()
                print("Couldn't load backbone, using random weights. Error: ", e)

            # try decoder
            try:
                w_dict = torch.load(path + "/segmentation_decoder" + path_append,
                                    map_location=lambda storage, loc: storage)
                self.decoder.load_state_dict(w_dict, strict=True)
                print("Successfully loaded model decoder weights")
            except Exception as e:
                print("Couldn't load decoder, using random weights. Error: ", e)

            # try head
            try:
                w_dict = torch.load(path + "/segmentation_head" + path_append,
                                    map_location=lambda storage, loc: storage)
                self.head.load_state_dict(w_dict, strict=True)
                print("Successfully loaded model head weights")
            except Exception as e:
                print("Couldn't load head, using random weights. Error: ", e)

            # try CRF
            if self.CRF:
                try:
                    w_dict = torch.load(path + "/segmentation_CRF" + path_append,
                                        map_location=lambda storage, loc: storage)
                    self.CRF.load_state_dict(w_dict, strict=True)
                    print("Successfully loaded model CRF weights")
                except Exception as e:
                    print("Couldn't load CRF, using random weights. Error: ", e)
        else:
            print("No path to pretrained, using random init.")

    def forward(self, x, mask=None):
        y, skips = self.backbone(x)
        y = self.decoder(y, skips)
        y = self.head(y)
        y = F.softmax(y, dim=1)
        if self.CRF:
            assert (mask is not None)
            y = self.CRF(x, y, mask)
        return y

    def save_checkpoint(self, logdir, suffix=""):
        # Save the weights
        torch.save(self.backbone.state_dict(), logdir +
                   "/backbone" + suffix)
        torch.save(self.decoder.state_dict(), logdir +
                   "/segmentation_decoder" + suffix)
        torch.save(self.head.state_dict(), logdir +
                   "/segmentation_head" + suffix)
        if self.CRF:
            torch.save(self.CRF.state_dict(), logdir +
                       "/segmentation_CRF" + suffix)
