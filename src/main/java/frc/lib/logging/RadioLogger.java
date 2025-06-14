// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.logging;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.net.http.HttpResponse.BodyHandlers;
import java.time.Duration;
import java.util.concurrent.CancellationException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;

import org.json.simple.JSONObject;

import com.fasterxml.jackson.databind.util.JSONPObject;
import com.pathplanner.lib.util.JSONUtil;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Class to poll the /status endpoint of a radio and put the results on NetworkTables.
 * This class assumes NetworkTables is already being auto-logged.
 */
public class RadioLogger {

    // Target radio IP
    private String target;

    // Timing intervals for querying status in microseconds
    // This does not include the time taken for the request to arrive
    private long pollRate = 250000;

    // Timeout length for when the endpoint doesn't respond, in milliseconds
    private long timeout = 2000;


    private HttpClient client = HttpClient.newHttpClient();
    private HttpRequest request;

    @Logged
    private long lastResultsTimestamp = RobotController.getFPGATime();
    private boolean requestInFlight = false;
    private CompletableFuture<String> liveRequest;

    private String latestResults = "Initializing";

    public RadioLogger(String ip)
    {
        target = ip;

        request = HttpRequest.newBuilder()
         .uri(URI.create(target))
         .timeout(Duration.ofMillis(timeout))
         .header("Accept","application/json")
         .build();

    }


    private void query()
    {
        /*client.sendAsync(request, BodyHandlers.ofString())
         .thenApply(HttpResponse::body)
         .thenAccept((str) -> SmartDashboard.putString("radioLog", str))
         .join();
        */

        /*client.sendAsync(request, BodyHandlers.ofString())
        
        .thenApply(HttpResponse::body)
        .thenAccept((str) -> SmartDashboard.putString("radioLog", str))
        .exceptionally(this::handleException)
        
        ;*/
        //.join();

        requestInFlight = true;
        liveRequest = client.sendAsync(request, BodyHandlers.ofString())
            .thenApply(HttpResponse::body);
    }

    private Void handleException(Throwable t)
    {
        DriverStation.reportError("Failed to query "+target+": "+t.getLocalizedMessage(), t.getStackTrace());
        SmartDashboard.putString("radioLog", t.getLocalizedMessage());
        return null;// Needs OBJECT Void, not void void
    }

    private void handleLog(String str)
    {
        
    }

    private void checkResult()
    {
        if(liveRequest.isDone())
        {
            requestInFlight = false;
            lastResultsTimestamp = RobotController.getFPGATime();

            try {
                latestResults = liveRequest.get();
            } catch (InterruptedException | ExecutionException | CancellationException e) {
                latestResults = e.getLocalizedMessage();
            }
        }
    }

   

    @Logged
    public String logData()
    {
        // If a query is in flight, check if it is done
        if(requestInFlight)
        {
            checkResult();
        }
        // Check if we need to send another request
        else if(!requestInFlight && RobotController.getFPGATime() > lastResultsTimestamp+pollRate)
        {
            query();
        }

        return latestResults;
    }

   
    
}

