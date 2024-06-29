using UnityEngine;
using System;
using System.Collections.Generic;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;
using Map;

public class MapVisualizer : MonoBehaviour
{
    public Material navigablePolygonMaterial;
    public Material nonNavigablePolygonMaterial;
    public Material pathMaterial;
    public GameObject nonNavigablePrefab;
    public GameObject startPointPrefab;
    public GameObject endPointPrefab;
    public GameObject planePrefab;

    private GameObject currentStartPointObject;
    private GameObject currentEndPointObject;
    private List<GameObject> pathInstances = new List<GameObject>();

    public void VisualizeMap(IndoorSpace indoorSpace)
    {
        foreach (var cellSpace in indoorSpace.CellSpaces)
        {
            CreateSpaceVisualization(cellSpace.Space as Polygon, (bool)cellSpace.Properties["navigable"]);
        }
    }

    public void CreateSpaceVisualization(Polygon polygon, bool navigation)
    {
        GameObject polygonObject = Instantiate(planePrefab, Vector3.zero, Quaternion.identity, transform);
        polygonObject.name = "Polygon";
        polygonObject.transform.position = new Vector3((float)polygon.Centroid.X, 0, (float)polygon.Centroid.Y);
        polygonObject.transform.localScale = new Vector3(0.1f, 1f, 0.1f); // Adjust based on actual size requirements

        MeshRenderer renderer = polygonObject.GetComponent<MeshRenderer>();
        renderer.material = navigation ? navigablePolygonMaterial : nonNavigablePolygonMaterial;
        
        if (!navigation)
        {
            GameObject shelfObject = Instantiate(nonNavigablePrefab, Vector3.zero, Quaternion.identity, transform);
            shelfObject.name = "Shelf";
            shelfObject.transform.position = new Vector3((float)polygon.Centroid.X, 0.5f, (float)polygon.Centroid.Y);
            shelfObject.transform.localScale = new Vector3(1f, 1f, 1f); // Adjust based on actual size requirements
        }
    }
}
