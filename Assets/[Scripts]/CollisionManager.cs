using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class CollisionManager : MonoBehaviour
{
    public CubeBehaviour[] cubes;
    public BulletBehaviour[] bullets;

    private static Vector3[] faces;

    // Start is called before the first frame update
    void Start()
    {
        cubes = FindObjectsOfType<CubeBehaviour>();

        faces = new Vector3[]
        {
            Vector3.left, Vector3.right,
            Vector3.down, Vector3.up,
            Vector3.back , Vector3.forward
        };
    }

    // Update is called once per frame
    void Update()
    {
        bullets = FindObjectsOfType<BulletBehaviour>();

        // check each AABB with every other AABB in the scene
        for (int i = 0; i < cubes.Length; i++)
        {
            for (int j = 0; j < cubes.Length; j++)
            {
                if (i != j)
                {
                    CheckAABBs(cubes[i], cubes[j]);
                    AABBAABBCollision(cubes[i], cubes[j]);
                }
            }
        }

        // Check each sphere against each AABB in the scene
        foreach (var bullet in bullets)
        {
            foreach (var cube in cubes)
            {
                if (cube.name != "Player")
                {
                    CheckBulletAABB(bullet, cube);
                }
                
            }
        }


    }

    public static void CheckBulletAABB(BulletBehaviour bullet, CubeBehaviour b)
    {
        // get box closest point to sphere center by clamping
        var x = Mathf.Max(b.min.x, Mathf.Min(bullet.transform.position.x, b.max.x));
        var y = Mathf.Max(b.min.y, Mathf.Min(bullet.transform.position.y, b.max.y));
        var z = Mathf.Max(b.min.z, Mathf.Min(bullet.transform.position.z, b.max.z));

        var distance = Math.Sqrt((x - bullet.transform.position.x) * (x - bullet.transform.position.x) +
                                 (y - bullet.transform.position.y) * (y - bullet.transform.position.y) +
                                 (z - bullet.transform.position.z) * (z - bullet.transform.position.z));

        if ((distance < bullet.size.x && distance < bullet.size.y && distance < bullet.size.z) && (!bullet.isColliding))
        {
            // determine the distances between the contact extents
            float[] distances = {
                (b.max.x - bullet.transform.position.x),
                (bullet.transform.position.x - b.min.x),
                (b.max.y - bullet.transform.position.y),
                (bullet.transform.position.y - b.min.y),
                (b.max.z - bullet.transform.position.z),
                (bullet.transform.position.z - b.min.z)
            };

            float penetration = float.MaxValue;
            Vector3 face = Vector3.zero;

            // check each face to see if it is the one that connected
            for (int i = 0; i < 6; i++)
            {
                if (distances[i] < penetration)
                {
                    // determine the penetration distance
                    penetration = distances[i];
                    face = faces[i];
                }
            }

            bullet.penetration = penetration;
            bullet.collisionNormal = face;
            //s.isColliding = true;

            
            Reflect(bullet);
        }

    }
    
    // This helper function reflects the bullet when it hits an AABB face
    private static void Reflect(BulletBehaviour bullet)
    {
        if ((bullet.collisionNormal == Vector3.forward) || (bullet.collisionNormal == Vector3.back))
        {
            bullet.direction = new Vector3(bullet.direction.x, bullet.direction.y, -bullet.direction.z);
        }
        else if ((bullet.collisionNormal == Vector3.right) || (bullet.collisionNormal == Vector3.left))
        {
            bullet.direction = new Vector3(-bullet.direction.x, bullet.direction.y, bullet.direction.z);
        }
        else if ((bullet.collisionNormal == Vector3.up) || (bullet.collisionNormal == Vector3.down))
        {
            bullet.direction = new Vector3(bullet.direction.x, -bullet.direction.y, bullet.direction.z);
        }
    }


    public static void CheckAABBs(CubeBehaviour a, CubeBehaviour b)
    {
        Contact contactB = new Contact(b);

        if ((a.min.x <= b.max.x && a.max.x >= b.min.x) &&
            (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
            (a.min.z <= b.max.z && a.max.z >= b.min.z))
        {
            // determine the distances between the contact extents
            float[] distances = {
                (b.max.x - a.min.x),
                (a.max.x - b.min.x),
                (b.max.y - a.min.y),
                (a.max.y - b.min.y),
                (b.max.z - a.min.z),
                (a.max.z - b.min.z)
            };

            float penetration = float.MaxValue;
            Vector3 face = Vector3.zero;

            // check each face to see if it is the one that connected
            for (int i = 0; i < 6; i++)
            {
                if (distances[i] < penetration)
                {
                    // determine the penetration distance
                    penetration = distances[i];
                    face = faces[i];
                }
            }
            
            // set the contact properties
            contactB.face = face;
            contactB.penetration = penetration;


            // check if contact does not exist
            if (!a.contacts.Contains(contactB))
            {
                // remove any contact that matches the name but not other parameters
                for (int i = a.contacts.Count - 1; i > -1; i--)
                {
                    if (a.contacts[i].cube.name.Equals(contactB.cube.name))
                    {
                        a.contacts.RemoveAt(i);
                    }
                }

                if (contactB.face == Vector3.down)
                {
                    a.gameObject.GetComponent<RigidBody3D>().Stop();
                    a.isGrounded = true;
                }
                

                // add the new contact
                a.contacts.Add(contactB);
                a.isColliding = true;
                
            }
        }
        else
        {

            if (a.contacts.Exists(x => x.cube.gameObject.name == b.gameObject.name))
            {
                a.contacts.Remove(a.contacts.Find(x => x.cube.gameObject.name.Equals(b.gameObject.name)));
                a.isColliding = false;

                if (a.gameObject.GetComponent<RigidBody3D>().bodyType == BodyType.DYNAMIC)
                {
                    a.gameObject.GetComponent<RigidBody3D>().isFalling = true;
                    a.isGrounded = false;
                }
            }
        }
    }

    static void AABBAABBCollision(CubeBehaviour objectA, CubeBehaviour objectB)
    {
        //Setup for all values necessary for collision
        Vector3 halfSizeA = objectA.GetHalfSize();
        Vector3 halfSizeB = objectB.GetHalfSize();

        Vector3 distanceAB = objectB.transform.position - objectA.transform.position;

        float distanceX = Mathf.Abs(distanceAB.x);
        float distanceY = Mathf.Abs(distanceAB.y);
        float distanceZ = Mathf.Abs(distanceAB.z);


        float penetrationX = halfSizeA.x + halfSizeB.x - distanceX;
        float penetrationY = halfSizeA.y + halfSizeB.y - distanceY;
        float penetrationZ = halfSizeA.z + halfSizeB.z - distanceZ;

        //Check for collision here
        if (penetrationX < 0 || penetrationY < 0 || penetrationZ < 0)
        {
            return;
        }

        // Find minimumTraslationVector (i.e. what is the shortest path we can take)
        // Along which axis are they closest to being seperate
        // Move along that axis according to how much overlap there is

        Vector3 contact;
        Vector3 collisionNormalAtoB;
        Vector3 minimumTranslationVector;

        if (penetrationX < penetrationY && penetrationX < penetrationZ) // is penX the shortest?
        {
            collisionNormalAtoB = new Vector3(Mathf.Sign(distanceAB.x), 0, 0);    // Sign returns -1 or 1 based on sign
            minimumTranslationVector = collisionNormalAtoB * penetrationX;
        }
        else if (penetrationY < penetrationX && penetrationY < penetrationZ) // is penY the shortest?
        {
            collisionNormalAtoB = new Vector3(0, Mathf.Sign(distanceAB.y), 0);    // Sign returns -1 or 1 based on sign
            minimumTranslationVector = collisionNormalAtoB * penetrationY;
        }
        else //if (penetrationZ < penetrationY && penetrationZ < penetrationX) // is penZ the shortest?   // could just be else
        {
            collisionNormalAtoB = new Vector3(0, 0, Mathf.Sign(distanceAB.z));    // Sign returns -1 or 1 based on sign
            minimumTranslationVector = collisionNormalAtoB * penetrationZ;
        }

        contact = objectA.transform.position + minimumTranslationVector;

        ApplyMinimumTraslationVector(objectA.gameObject.GetComponent<RigidBody3D>(), objectB.gameObject.GetComponent<RigidBody3D>(), minimumTranslationVector, collisionNormalAtoB, contact);

    }

    static void ApplyMinimumTraslationVector(RigidBody3D a, RigidBody3D b, Vector3 minimumTranslationVectorAtoB, Vector3 collisionNormalAtoB, Vector3 contact)
    {
        //calculate the proper scaler if object is locked or not
        ComputeMovementScalars(a, b, out float moveScalarA, out float moveScalarB);

        // calculate Translations
        Vector3 TranslationVectorA = -minimumTranslationVectorAtoB * moveScalarA;
        Vector3 TranslationVectorB = minimumTranslationVectorAtoB * moveScalarB;

        // Update Positions based on Translations
        a.transform.Translate(TranslationVectorA);
        b.transform.Translate(TranslationVectorB);

        Vector3 contactPoint = contact;

        ApplyVelocityResponse(a, b, collisionNormalAtoB);

    }

    static void ComputeMovementScalars(RigidBody3D a, RigidBody3D b, out float mtvScalarA, out float mtvScalarB)
    {
        // Check to see if either object is Locked
        if (a.bodyType == BodyType.STATIC && b.bodyType == BodyType.DYNAMIC)
        {
            //if A is locked and B is not
            //mtvScalarA = 0.0f;
            //mtvScalarB = 1.0f;
            //return;
        }
        if (a.bodyType == BodyType.DYNAMIC && b.bodyType == BodyType.STATIC)
        {
            //if B is locked and A is not
            //mtvScalarA = 1.0f;
            //mtvScalarB = 0.0f;
            //return;
        }
        if (a.bodyType == BodyType.DYNAMIC && b.bodyType == BodyType.DYNAMIC)
        {
            //if both objects are not locked
            mtvScalarA = 0.5f;
            mtvScalarB = 0.5f;
            return;
        }
        //else is A and B is locked
        mtvScalarA = 0.0f;
        mtvScalarB = 0.0f;
    }

    static void ApplyVelocityResponse(RigidBody3D objA, RigidBody3D objB, Vector3 collisionNormal)
    {
        Vector3 normal = collisionNormal;

        // Velocity of B relative to A
        Vector3 relativeVelocityAB = objB.velocity - objA.velocity;

        // Find relative velocity
        float relativeNormalVelocityAB = Vector3.Dot(relativeVelocityAB, normal);

        // Early exit if they are not going towards each other (no bounce)
        if (relativeNormalVelocityAB >= 0.0f)
        {
            return;
        }

        // Choose a coefficient of restitution
        float restitution = (0.0f + 0.0f) * 0.5f;

        float deltaV;

        float minimumRelativeVelocityForBounce = 3.0f;

        // If we only need the objects to slide and not bounce, then...
        if (relativeNormalVelocityAB < -minimumRelativeVelocityForBounce)
        {
            // Determine change in velocity 
            deltaV = (relativeNormalVelocityAB * (1.0f + restitution));
        }
        else
        {
            // no bounce
            deltaV = (relativeNormalVelocityAB);
        }

        float impulse;
        // respond differently based on locked states
        if (objA.bodyType == BodyType.STATIC && objB.bodyType == BodyType.DYNAMIC)
        {
            // Only B
            impulse = -deltaV * objB.mass;
            objB.velocity += normal * (impulse / (objB.mass));
        }
        else if (objA.bodyType == BodyType.DYNAMIC && objB.bodyType == BodyType.STATIC)
        {
            // impulse required to creat our desired change in velocity
            // impulse = Force * time = kg * m/s^2 * s = kg m/s
            // impulse / objA.mass == deltaV
            // Only A change velocity
            impulse = -deltaV * objA.mass;
            objA.velocity -= normal * (impulse / (objA.mass));
        }
        else if (objA.bodyType == BodyType.DYNAMIC && objB.bodyType == BodyType.DYNAMIC)
        {
            // Both
            impulse = deltaV / ((1.0f / objA.mass) + (1.0f / objB.mass));
            objA.velocity += normal * (impulse / objA.mass);
            objB.velocity -= normal * (impulse / objB.mass);
        }
        else if (objA.bodyType == BodyType.STATIC && objB.bodyType == BodyType.STATIC)
        {
            // Nadda
        }
        else
        {
            return;
        }

        // subtract the component of relative velocity that is along the normal of the collision to receive the tangential velocity
        Vector3 relativeSurfaceVelocity = relativeVelocityAB - (relativeNormalVelocityAB * normal);

        //ApplyFriction(objA, objB, relativeSurfaceVelocity, normal);
    }
}
