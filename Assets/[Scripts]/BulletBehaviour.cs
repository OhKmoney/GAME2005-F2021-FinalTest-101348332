using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class BulletBehaviour : MonoBehaviour
{
    public float speed;
    public Vector3 direction;
    public float range;
    public bool debug;
    public bool isColliding;
    public Vector3 collisionNormal;
    public float penetration;

    public Vector3 size;

    public BulletManager bulletManager;

    // Start is called before the first frame update
    void Start()
    {
        size.x = transform.localScale.x;
        size.y = transform.localScale.y;
        size.z = transform.localScale.z;

        isColliding = false;
       
        bulletManager = FindObjectOfType<BulletManager>();
    }

    // Update is called once per frame
    void Update()
    {
        _Move();
        _CheckBounds();
    }

    private void _Move()
    {
        transform.position += direction * speed * Time.deltaTime;
    }

    private void _CheckBounds()
    {
        if (Vector3.Distance(transform.position, Vector3.zero) > range)
        {
            bulletManager.ReturnBullet(this.gameObject);
        }
    }

    private void OnDrawGizmos()
    {
        if (debug)
        {
            Gizmos.color = Color.magenta;

            Gizmos.DrawWireCube(transform.position, size);
        }
    }
}
