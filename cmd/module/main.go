package main

import (
	"obstaclespointclouddepth"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/vision"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(resource.APIModel{API: vision.API, Model: obstaclespointclouddepth.ObstaclesPointCloud},
					   resource.APIModel{API: vision.API, Model: obstaclespointclouddepth.ObstaclesDepth})
}
