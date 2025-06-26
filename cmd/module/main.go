package main

import (
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/vision"

	obstaclespointcloud "obstaclespointcloud/obstacles-pointcloud"
)

func main() {
	module.ModularMain(resource.APIModel{
		API:   vision.API,
		Model: obstaclespointcloud.Model,
	})
}
